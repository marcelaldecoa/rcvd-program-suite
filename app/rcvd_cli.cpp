// app/rcvd_cli.cpp — the command-line front end, spec §4.1 and §16.2.
//
// Rule A1: any behaviour reachable through the GUI SHALL be reachable through
// rcvd-cli and through the C API with IDENTICAL results, bit-for-bit (§18.3). So
// this file contains no equation and no unit conversion beyond calling io/display.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rcvd/analysis/lap_solver.hpp"
#include "rcvd/io/csv.hpp"
#include "rcvd/io/display.hpp"
#include "rcvd/io/document.hpp"
#include "rcvd/io/manifest.hpp"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/models/four_wheeler.hpp"

using namespace rcvd;

namespace {

int fail(const core::Diagnostic& d) {
    std::cerr << d.code << ": " << d.message << "\n";
    return 1;
}

void usage() {
    std::cout <<
        "rcvd-cli — Race Car Vehicle Dynamics Program Suite\n"
        "\n"
        "  rcvd-cli steady  --vehicle F --cf C --cr C --speed V [--steer D]\n"
        "  rcvd-cli derivs  --vehicle F --cf C --cr C --speed V\n"
        "  rcvd-cli lap     --vehicle F --cf C --cr C --speed V --steer D\n"
        "                   --inner R [--outer R] [--start R]\n"
        "\n"
        "Options\n"
        "  --vehicle FILE   vehicle document (JSON, §5.1)\n"
        "  --cf, --cr       axle cornering stiffness, negative (§3.3)\n"
        "  --speed          forward speed\n"
        "  --steer          road-wheel steer angle\n"
        "  --units si|us    DISPLAY system only; the core always computes in SI\n"
        "                   (Rule U1). Default si.\n"
        "  --in-units U     unit for the numeric options above, e.g. ft, mph, deg\n"
        "  --digits N       significant figures, 1..17 (default 6). §18.3 requires the\n"
        "                   GUI, the CLI and the C API to agree BIT-FOR-BIT; at the\n"
        "                   default 6 figures that cannot be checked from this\n"
        "                   output, so --digits 17 round-trips a binary64 exactly.\n"
        "  --manifest FILE  write the §4.5 run manifest here\n"
        "  --csv FILE       write the trajectory as CSV (lap only)\n"
        "\n"
        "All numeric options are read in the --in-units system and printed in the\n"
        "--units system; every value between those two boundaries is SI.\n";
}

struct Args {
    std::string command, vehicle_file, manifest_file, csv_file;
    io::UnitSystem units = io::UnitSystem::si;
    io::UnitSystem in_units = io::UnitSystem::si;
    double cf = 0, cr = 0, speed = 0, steer = 0;
    double inner = 0, outer = 0, start = 0;
    bool have_outer = false, have_start = false;
    int digits = 6;
};

bool read_file(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

// The input boundary for command-line numbers. Same discipline as §5's documents:
// a number arrives in a named unit and is SI from here on.
double to_si_in(double v, units::Dim dim, io::UnitSystem sys) {
    const io::DisplayUnits d = io::DisplayUnits::of(sys);
    return *units::to_si(v, d.unit_for(dim), dim);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0) {
        usage();
        return argc < 2 ? 1 : 0;
    }

    Args a;
    a.command = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--vehicle")       a.vehicle_file = next();
        else if (k == "--manifest") a.manifest_file = next();
        else if (k == "--csv")      a.csv_file = next();
        else if (k == "--cf")       a.cf = std::stod(next());
        else if (k == "--cr")       a.cr = std::stod(next());
        else if (k == "--speed")    a.speed = std::stod(next());
        else if (k == "--steer")    a.steer = std::stod(next());
        else if (k == "--inner")    a.inner = std::stod(next());
        else if (k == "--outer")  { a.outer = std::stod(next()); a.have_outer = true; }
        else if (k == "--start")  { a.start = std::stod(next()); a.have_start = true; }
        else if (k == "--digits")   a.digits = std::stoi(next());
        else if (k == "--units" || k == "--in-units") {
            const auto s = io::parse_unit_system(next());
            if (!s) { std::cerr << "unknown unit system\n"; return 1; }
            (k == "--units" ? a.units : a.in_units) = *s;
        } else {
            std::cerr << "unknown option " << k << "\n";
            return 1;
        }
    }

    if (a.digits < 1 || a.digits > 17) {
        std::cerr << "--digits must be in 1..17 (17 round-trips a binary64 exactly)\n";
        return 1;
    }
    if (a.vehicle_file.empty()) {
        std::cerr << "--vehicle is required\n";
        return 1;
    }
    std::string text;
    if (!read_file(a.vehicle_file, text)) {
        std::cerr << "cannot read " << a.vehicle_file << "\n";
        return 1;
    }
    const auto doc = io::parse_json(text);
    if (!doc.ok()) return fail(doc.error());
    const auto vin = io::load_vehicle(doc.value());
    if (!vin.ok()) return fail(vin.error());
    const auto vr = core::resolve(vin.value());
    if (!vr.ok()) return fail(vr.error());
    const core::Vehicle& veh = vr.value();
    for (const auto& w : veh.warnings) std::cerr << w.code << ": " << w.message << "\n";

    // Command-line numbers cross into SI here, once.
    const models::AxleStiffness C{
        to_si_in(a.cf, units::Dim::CorneringStiffness, a.in_units),
        to_si_in(a.cr, units::Dim::CorneringStiffness, a.in_units)};
    const double V = to_si_in(a.speed, units::Dim::Speed, a.in_units);
    const double delta = to_si_in(a.steer, units::Dim::Angle, a.in_units);

    // Width follows precision so that --digits 17 still produces aligned columns.
    const int w = a.digits + 8;

    const io::DisplayUnits disp = io::DisplayUnits::of(a.units);
    const auto show = [&](double si, units::Dim dim) {
        return *io::to_display(si, dim, disp);
    };
    const auto unit = [&](units::Dim dim) { return std::string(disp.unit_for(dim)); };

    std::printf("# rcvd-cli %s — vehicle '%s', display units %s\n", a.command.c_str(),
                veh.name.c_str(), std::string(io::to_string(a.units)).c_str());

    if (a.command == "derivs") {
        const auto d = models::stability_derivatives(veh, C, V);
        // §8.3's six derivatives span four dimensions; each is displayed through
        // the unit its own dimension resolves to, never a shared divisor.
        std::printf("  Y_beta   %*.*g  %s\n", w, a.digits, show(d.Y_beta, units::Dim::CorneringStiffness),
                    unit(units::Dim::CorneringStiffness).c_str());
        std::printf("  Y_delta  %*.*g  %s\n", w, a.digits, show(d.Y_delta, units::Dim::CorneringStiffness),
                    unit(units::Dim::CorneringStiffness).c_str());
        std::printf("  N_beta   %*.*g  %s\n", w, a.digits, show(d.N_beta, units::Dim::AngularStiffness),
                    unit(units::Dim::AngularStiffness).c_str());
        std::printf("  N_delta  %*.*g  %s\n", w, a.digits, show(d.N_delta, units::Dim::AngularStiffness),
                    unit(units::Dim::AngularStiffness).c_str());
        // Y_r (N*s/rad) and N_r (N*m*s/rad) are their OWN dimensions, not the
        // stiffnesses they are so often displayed with. Each resolves its own unit,
        // which is what makes B-04 and B-07 expressible in US format at all.
        std::printf("  Y_r      %*.*g  %s\n", w, a.digits, show(d.Y_r, units::Dim::SideForceDamping),
                    unit(units::Dim::SideForceDamping).c_str());
        std::printf("  N_r      %*.*g  %s\n", w, a.digits, show(d.N_r, units::Dim::YawDamping),
                    unit(units::Dim::YawDamping).c_str());
        const auto m = models::handling_metrics(veh, C);
        // Rule A1: the deg/g normalisation is a core function, not arithmetic here.
        std::printf("  K_us     %*.*g  deg/g\n", w, a.digits,
                    models::understeer_gradient_deg_per_g(veh, m));
        std::printf("  NSP      %*.*g  -\n", w, a.digits, m.NSP);
        std::printf("  SM       %*.*g  -\n", w, a.digits, m.SM);
        if (m.V_ch)   std::printf("  V_ch     %*.*g  %s\n", w, a.digits,
                                  show(*m.V_ch, units::Dim::Speed), unit(units::Dim::Speed).c_str());
        if (m.V_crit) std::printf("  V_crit   %*.*g  %s\n", w, a.digits,
                                  show(*m.V_crit, units::Dim::Speed), unit(units::Dim::Speed).c_str());
        std::printf("  V_tan    %*.*g  %s\n", w, a.digits, show(m.V_tan, units::Dim::Speed),
                    unit(units::Dim::Speed).c_str());
        return 0;
    }

    if (a.command == "steady") {
        const auto ss = models::steady_state(veh, C, V, delta);
        if (!ss.ok()) return fail(ss.error());
        std::printf("  speed    %*.*g  %s\n", w, a.digits, show(V, units::Dim::Speed),
                    unit(units::Dim::Speed).c_str());
        std::printf("  steer    %*.*g  %s\n", w, a.digits, show(delta, units::Dim::Angle),
                    unit(units::Dim::Angle).c_str());
        std::printf("  beta     %*.*g  %s\n", w, a.digits, show(ss.value().beta_rad, units::Dim::Angle),
                    unit(units::Dim::Angle).c_str());
        std::printf("  yaw rate %*.*g  %s/s\n", w, a.digits,
                    ss.value().r_rad_per_s / (a.units == io::UnitSystem::us ? units::DEGREE : 1.0),
                    unit(units::Dim::Angle).c_str());
        std::printf("  A_Y      %*.*g  %s\n", w, a.digits,
                    show(ss.value().A_Y_mps2, units::Dim::Acceleration),
                    unit(units::Dim::Acceleration).c_str());
        std::printf("  rcond    %*.*g  -\n", w, a.digits, ss.value().rcond);
        return 0;
    }

    if (a.command == "lap") {
        analysis::Circuit circuit;
        circuit.inner_radius = to_si_in(a.inner, units::Dim::Length, a.in_units);
        circuit.outer_radius = a.have_outer
                                   ? to_si_in(a.outer, units::Dim::Length, a.in_units)
                                   : std::numeric_limits<double>::infinity();
        const double start_r = a.have_start
                                   ? to_si_in(a.start, units::Dim::Length, a.in_units)
                                   : circuit.inner_radius * 1.5;

        analysis::DriverPlan plan;
        plan.initial_speed_mps = V;
        plan.steer = models::Maneuver::make_fixed(delta);
        plan.start_x = start_r;
        plan.start_heading_rad = units::PI / 2.0;

        models::BicyclePlant plant(veh, C, V);
        const auto m = models::handling_metrics(veh, C);
        analysis::TransientContext ctx;
        ctx.tau_dominant_s = models::eigenstructure(veh, C, V).tau_dominant;
        ctx.steady_state_radius_m =
            delta != 0.0 ? V / (models::yaw_rate_gain(veh, m.K, V) * delta)
                         : std::numeric_limits<double>::infinity();

        analysis::LapConfig cfg;
        cfg.calculation_length_s = 120.0;
        const auto res = analysis::lap_solve(plant, circuit, plan, cfg, ctx);
        if (!res.ok()) return fail(res.error());

        std::printf("  event    %14s\n", analysis::to_string(res.value().event).c_str());
        // Lap time followed --digits like everything else once §18.3 had to be
        // checkable: a 9-figure lap time cannot demonstrate bit-for-bit agreement
        // either, and the event-location tolerance of §11.4 is 1e-9 s.
        std::printf("  time     %*.*g  s\n", w, std::max(a.digits, 9), res.value().time_s);
        std::printf("  steps    %14ld accepted, %ld rejected\n", res.value().stats.accepted,
                    res.value().stats.rejected);
        for (const auto& d : res.value().diagnostics) {
            std::printf("  %s: %s\n", std::string(d.code).c_str(), d.message.c_str());
        }

        if (!a.csv_file.empty()) {
            std::vector<io::Channel> ch;
            const auto column = [&](std::string name, units::Dim dim, auto pick) {
                io::Channel c;
                c.name = std::move(name);
                c.dim = dim;
                c.display_unit = std::string(disp.unit_for(dim));
                for (const auto& s : res.value().path) c.si_values.push_back(pick(s));
                ch.push_back(std::move(c));
            };
            column("time", units::Dim::Time, [](const auto& s) { return s.t; });
            column("X_E", units::Dim::Length, [](const auto& s) { return s.X_E; });
            column("Y_E", units::Dim::Length, [](const auto& s) { return s.Y_E; });
            column("psi", units::Dim::Angle, [](const auto& s) { return s.psi; });
            column("v", units::Dim::Speed, [](const auto& s) { return s.v; });
            const auto csv = io::write_csv(ch);
            if (!csv.ok()) return fail(csv.error());
            std::ofstream out(a.csv_file, std::ios::binary);
            out << csv.value();
            std::printf("  wrote %s (%zu records, %s units)\n", a.csv_file.c_str(),
                        res.value().path.size(), std::string(io::to_string(a.units)).c_str());
        }

        if (!a.manifest_file.empty()) {
            io::RunManifest man;
            man.build.compiler = "cl";
            man.build.platform = "windows-x86_64";
            man.resolved_input = io::resolved_vehicle_json(veh);
            man.integrator = res.value().stats;
            man.gravity_mps2 = veh.g;
            man.diagnostics = res.value().diagnostics;
            std::ofstream out(a.manifest_file, std::ios::binary);
            out << io::manifest_json(man).dump();
            std::printf("  wrote %s\n", a.manifest_file.c_str());
        }
        return 0;
    }

    std::cerr << "unknown command '" << a.command << "'\n";
    usage();
    return 1;
}
