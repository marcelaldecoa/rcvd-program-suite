// app/gui/program_spec.hpp — what each of P1...P6 puts on screen.
//
// Rule A1 is the whole design of this file: "`app` SHALL contain no equation, no
// unit conversion beyond calling `units`, and no numerical algorithm." So every
// run function below is a sequence of calls into `models`, `analysis` and `tire`,
// and every number that reaches the screen goes through io::to_display. There is
// no arithmetic on physical quantities here.
//
// §16.1 wants one window per program from a common shell. Rather than six
// hand-written windows that drift apart, each program is DESCRIBED here and a
// single generic window renders the description. That is also what makes §16.2's
// "display the equivalent CLI command" mechanical rather than hand-maintained.
#ifndef RCVD_GUI_PROGRAM_SPEC_HPP
#define RCVD_GUI_PROGRAM_SPEC_HPP

#include <QString>

#include <array>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rcvd/analysis/lap_solver.hpp"
#include "rcvd/analysis/moment_method.hpp"
#include "rcvd/analysis/ride_roll.hpp"
#include "rcvd/core/vehicle.hpp"
#include "rcvd/io/display.hpp"
#include "rcvd/io/document.hpp"
#include "rcvd/io/json.hpp"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/models/four_wheeler.hpp"
#include "rcvd/tire/linear.hpp"
#include "rcvd/units/convert.hpp"

#include "plot_widget.hpp"

namespace rcvd::gui {

// Which block of the §5.1 vehicle document a field belongs to. The GUI builds a
// real vehicle document and hands it to io::load_vehicle, so the input path is
// the SAME code the CLI runs — which is how §18.3's bit-for-bit agreement is had
// by construction rather than by testing afterwards.
enum class Block { inertia, geometry, springing, steering, run };

struct FieldSpec {
    std::string key;            // document key, or CLI option name for run fields
    std::string label;
    units::Dim dim = units::Dim::Dimensionless;
    Block block = Block::run;
    double default_si = 0.0;
    // §5.1 writes front_weight_fraction and the steering ratio as bare numbers,
    // not {value, unit} objects. Everything else is a quantity.
    bool bare_number = false;
    // A model SWITCH, not a quantity: rendered as a checkbox, since §16.3's
    // "every numeric field carries a unit selector" has nothing to say about a
    // flag, and a 0/1 spin box with a "-" selector would misrepresent its nature.
    bool is_switch = false;
    bool switch_default = false;
    // §16.3: "Fields inert for the current model (e.g. CG height in P1) are
    // disabled with a tooltip saying why, never merely ignored." Non-empty here
    // means exactly that, and the text is the tooltip.
    std::string inert_reason;
    std::string us_unit;        // selector preset when the window is in US display
    // The field uses zero as "let the program choose" (an optional outer wall, a
    // start radius defaulting to 1.5 x inner). Such a field must be OMITTED from
    // the §16.2 command line, not emitted as "--start 0": rcvd-cli reads a
    // supplied zero literally, which for a start radius puts the car inside the
    // inner wall. The §18.3 gate caught exactly that.
    bool zero_means_default = false;
};

struct ResultRow {
    std::string name;
    double value = 0.0;
    units::Dim dim = units::Dim::Dimensionless;
    // Non-empty means `value` is already final and is labelled with this string
    // verbatim. Used for the quantities §3.4's vocabulary deliberately has no
    // entry for — the stability factor K (s^2/m^2), the MMM indices, rcond — so
    // that nothing here can masquerade as a unit conversion.
    std::string raw_unit;
    // A CATEGORICAL result — a limit character, a termination event. Not a number,
    // so it carries no unit and must not be formatted as one.
    std::string text;
    std::string note;
};

inline ResultRow si_row(std::string name, double si, units::Dim dim, std::string note = {}) {
    return {std::move(name), si, dim, {}, {}, std::move(note)};
}
inline ResultRow raw_row(std::string name, double v, std::string unit, std::string note = {}) {
    return {std::move(name), v, units::Dim::Dimensionless, std::move(unit), {},
            std::move(note)};
}
inline ResultRow text_row(std::string name, std::string text, std::string note = {}) {
    return {std::move(name), 0.0, units::Dim::Dimensionless, {}, std::move(text),
            std::move(note)};
}
// An angular RATE. rad/s -> deg/s is the same scale factor as rad -> deg, so this
// is a units call with the per-second carried in the label, not arithmetic.
inline ResultRow rate_row(std::string name, double si_rad_per_s, const io::DisplayUnits& d) {
    const std::string u(d.unit_for(units::Dim::Angle));
    return raw_row(std::move(name), *units::from_si(si_rad_per_s, u, units::Dim::Angle), u + "/s");
}

struct RunOutput {
    std::vector<ResultRow> rows;
    std::vector<Series> series;
    QString x_name = "x", x_unit = "-", y_name = "y", y_unit = "-";
    QString annotation;
    std::vector<core::Diagnostic> diagnostics;
    std::optional<core::Diagnostic> failure;   // Rule A3: reported, never thrown
};

using Inputs = std::map<std::string, double>;  // run-field key -> SI value

struct ProgramSpec {
    std::string id;             // "P1"
    std::string title;
    std::string section;        // the spec section this window implements
    std::string cli_command;    // rcvd-cli subcommand, empty when none exposes it
    std::vector<FieldSpec> vehicle_fields;
    std::vector<FieldSpec> run_fields;
    std::function<RunOutput(const core::Vehicle&, const Inputs&, const io::DisplayUnits&)> run;
};

// --------------------------------------------------------- the §5.1 document
//
// The GUI does NOT hand its field values straight to core::resolve. It writes a
// real vehicle document and feeds it through io::load_vehicle — the same function
// rcvd-cli calls on a file. Both front ends therefore share one input path, which
// is what makes §18.3's "identical results, bit-for-bit" a property of the
// structure rather than something only a test can hope for.
//
// The SI unit name is looked up in the closed vocabulary rather than spelled out,
// so a dimension added later cannot be serialised with a unit that does not exist.
[[nodiscard]] inline std::string_view si_unit_name(units::Dim dim) {
    for (const units::UnitEntry& e : units::kVocabulary) {
        if (e.dim == dim && e.to_si == 1.0) return e.name;
    }
    return "-";
}

using ValueOf = std::function<double(const FieldSpec&)>;

inline double default_value(const FieldSpec& f) { return f.default_si; }

[[nodiscard]] inline io::Json vehicle_document(const ProgramSpec& spec, const ValueOf& value) {
    io::JsonObject root, inertia, geometry, springing, steering;
    root["schema_version"] = io::Json(std::string("1.0"));
    root["name"] = io::Json(spec.id + " GUI session");

    for (const FieldSpec& f : spec.vehicle_fields) {
        const double v = value(f);
        io::Json entry = f.bare_number
                             ? io::Json(v)
                             : io::Json(io::JsonObject{
                                   {"value", io::Json(v)},
                                   {"unit", io::Json(std::string(si_unit_name(f.dim)))}});
        switch (f.block) {
            case Block::inertia:   inertia[f.key] = std::move(entry); break;
            case Block::geometry:  geometry[f.key] = std::move(entry); break;
            case Block::springing: springing[f.key] = std::move(entry); break;
            case Block::steering:  steering[f.key] = std::move(entry); break;
            case Block::run:       break;
        }
    }
    root["inertia"] = io::Json(std::move(inertia));
    root["geometry"] = io::Json(std::move(geometry));
    if (!springing.empty()) root["springing"] = io::Json(std::move(springing));
    if (!steering.empty()) root["steering"] = io::Json(std::move(steering));
    return io::Json(std::move(root));
}

// Whether rcvd-cli accepts an option for this run field. Emitting one it does
// not accept would be worse than printing nothing, so the list is explicit and
// shared between the Run tab's §16.2 display and the --dump-cli path.
[[nodiscard]] inline bool cli_exposes(const ProgramSpec& spec, const std::string& key) {
    static const std::vector<std::string> kDerivs{"cf", "cr", "speed"};
    static const std::vector<std::string> kSteady{"cf", "cr", "speed", "steer"};
    static const std::vector<std::string> kLap{"cf",    "cr",    "speed",
                                               "steer", "inner", "outer", "start"};
    const std::vector<std::string>& allowed = spec.cli_command == "lap"      ? kLap
                                              : spec.cli_command == "steady" ? kSteady
                                                                             : kDerivs;
    for (const std::string& k : allowed) {
        if (k == key) return true;
    }
    return false;
}

// ---------------------------------------------------------------- field recipes
namespace fields {

inline constexpr double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

// The §17.2 CH5 reference vehicle, as the opening state of every window: a
// configuration whose answers are published, so a user can tell at a glance
// whether the build is sane.
inline std::vector<FieldSpec> vehicle(bool roll_dof, bool springing, bool roll_centres) {
    const std::string no_roll =
        "the bicycle model of §8 has no roll degree of freedom, so this quantity "
        "cannot affect any output of this program";
    const std::string lateral =
        "this program is single-track; track width enters only the load-transfer "
        "and springing programs";

    std::vector<FieldSpec> f = {
        {"weight", "Weight W", units::Dim::Force, Block::inertia,
         3700.0 * units::POUND_FORCE, false, false, false, {}, "lbf"},
        {"yaw_inertia", "Yaw inertia I_z", units::Dim::Inertia, Block::inertia,
         1800.0 * units::SLUG_FT2_TO_KG_M2, false, false, false, {}, "slug*ft^2"},
        {"wheelbase", "Wheelbase l", units::Dim::Length, Block::geometry,
         104.5 * units::INCH, false, false, false, {}, "in"},
        {"front_weight_fraction", "Front weight fraction", units::Dim::Dimensionless,
         Block::geometry, 0.57, true, false, false, {}, "-"},
        {"cg_height", "CG height h", units::Dim::Length, Block::geometry,
         20.0 * units::INCH, false, false, false, roll_dof ? std::string{} : no_roll, "in"},
        {"track_front", "Front track t_F", units::Dim::Length, Block::geometry,
         60.0 * units::INCH, false, false, false, roll_dof ? std::string{} : lateral, "in"},
        {"track_rear", "Rear track t_R", units::Dim::Length, Block::geometry,
         60.0 * units::INCH, false, false, false, roll_dof ? std::string{} : lateral, "in"},
        {"ratio", "Steering ratio G_s", units::Dim::Dimensionless, Block::steering,
         14.0, true, false, false, {}, "-"},
    };
    // The roll-centre heights feed h2 = h_s - z_R_eff (9.4b), which P6's (12.11)
    // needs even though P6 supplies no roll stiffnesses. Omitting them silently
    // puts both roll centres at ground level and inflates m_s g h2 by the whole
    // sprung CG height.
    if (roll_centres) {
        f.push_back({"roll_centre_height_front", "Front roll centre z_RF", units::Dim::Length,
                     Block::geometry, 3.0 * units::INCH, false, false, false, {}, "in"});
        f.push_back({"roll_centre_height_rear", "Rear roll centre z_RR", units::Dim::Length,
                     Block::geometry, 5.0 * units::INCH, false, false, false, {}, "in"});
    }
    if (springing) {
        // Chassis-to-ground, per decision D-5. The band that keeps E-VEH-140 and
        // W-VEH-141 both reachable for CH5 is roughly 8.8...48 kN*m/rad.
        f.push_back({"roll_stiffness_chassis_to_ground_front", "Front roll stiffness K_phi_F",
                     units::Dim::AngularStiffness, Block::springing,
                     1100.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD, false, false, false, {},
                     "lbf*ft/deg"});
        f.push_back({"roll_stiffness_chassis_to_ground_rear", "Rear roll stiffness K_phi_R",
                     units::Dim::AngularStiffness, Block::springing,
                     700.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD, false, false, false, {},
                     "lbf*ft/deg"});
    }
    return f;
}

inline FieldSpec cf() {
    return {"cf", "Front axle C_F (negative, §3.3)", units::Dim::CorneringStiffness,
            Block::run, -640.0 * kLbfDeg, false, false, false, {}, "lbf/deg"};
}
inline FieldSpec cr() {
    return {"cr", "Rear axle C_R (negative, §3.3)", units::Dim::CorneringStiffness,
            Block::run, -580.0 * kLbfDeg, false, false, false, {}, "lbf/deg"};
}
inline FieldSpec speed(double si = 30.0 * units::MPH) {
    return {"speed", "Forward speed V", units::Dim::Speed, Block::run, si, false, false,
            false, {}, "mph"};
}
inline FieldSpec steer(double si = 2.0 * units::DEGREE) {
    return {"steer", "Road-wheel steer delta", units::Dim::Angle, Block::run, si, false, false,
            false, {}, "deg"};
}
// §9.7's algebraic loop. The default is `explicit`, "matching the source", and
// the two modes disagree near the limit, so the choice belongs on screen rather
// than buried in SolverSettings.
inline FieldSpec implicit_loop() {
    FieldSpec f;
    f.key = "implicit";
    f.label = "Implicit load-transfer loop (§9.7)";
    f.block = Block::run;
    f.is_switch = true;
    f.switch_default = false;
    return f;
}

inline FieldSpec breakaway() {
    return {"breakaway", "Breakaway slip alpha_bk", units::Dim::Angle, Block::run,
            5.0 * units::DEGREE, false, false, false, {}, "deg"};
}

}  // namespace fields

// ------------------------------------------------------------------ run helpers
namespace detail {

inline double in_or(const Inputs& v, const std::string& k, double fallback = 0.0) {
    const auto it = v.find(k);
    return it == v.end() ? fallback : it->second;
}

inline RunOutput fail(const core::Diagnostic& d) {
    RunOutput o;
    o.failure = d;
    return o;
}

// §8.7: "the path plot SHALL default to X_E on the VERTICAL axis and Y_E on the
// horizontal, with square axes". Getting it the other way round mirrors the
// trajectory, so a right-hand turn is drawn turning left. One function sets it up
// for both Speedway programs.
inline void set_path_axes(RunOutput& o, const io::DisplayUnits& d) {
    const QString u = QString::fromStdString(std::string(d.unit_for(units::Dim::Length)));
    o.x_name = "Y_E (east)";
    o.x_unit = u;
    o.y_name = "X_E (north)";
    o.y_unit = u;
    o.annotation = "§8.7 default: X_E vertical, Y_E horizontal, square axes.";
}

inline Series make_series(const QString& name, int style_index) {
    Series s;
    s.name = name;
    const auto styles = default_styles();
    s.style = styles[style_index % styles.size()];
    return s;
}

}  // namespace detail

// -------------------------------------------------------------------------- P1
inline RunOutput run_p1(const core::Vehicle& veh, const Inputs& in, const io::DisplayUnits& d) {
    using namespace models;
    RunOutput o;
    const AxleStiffness C{detail::in_or(in, "cf"), detail::in_or(in, "cr")};
    const double V = detail::in_or(in, "speed");
    const double delta = detail::in_or(in, "steer");

    const auto der = stability_derivatives(veh, C, V);
    const auto met = handling_metrics(veh, C);
    const auto eig = eigenstructure(veh, C, V);

    // §8.3's six derivatives span four distinct dimensions; each is displayed
    // through its own, never a shared divisor. That is the point of giving Y_r and
    // N_r their own vocabulary entries.
    o.rows.push_back(si_row("Y_beta", der.Y_beta, units::Dim::CorneringStiffness));
    o.rows.push_back(si_row("Y_r", der.Y_r, units::Dim::SideForceDamping));
    o.rows.push_back(si_row("Y_delta", der.Y_delta, units::Dim::CorneringStiffness));
    o.rows.push_back(si_row("N_beta", der.N_beta, units::Dim::AngularStiffness));
    o.rows.push_back(si_row("N_r", der.N_r, units::Dim::YawDamping));
    o.rows.push_back(si_row("N_delta", der.N_delta, units::Dim::AngularStiffness));

    o.rows.push_back(raw_row("K (8.14)", met.K, "s^2/m^2"));
    // Rule U4: g is a field constant, so deg/g is NOT a unit conversion and the
    // normalisation lives in `models`, not in `units` and not here.
    o.rows.push_back(raw_row("K_us (8.18)", understeer_gradient_deg_per_g(veh, met), "deg/g"));
    o.rows.push_back(raw_row("NSP (8.22)", met.NSP, "fraction of l aft of front axle"));
    o.rows.push_back(raw_row("SM (8.23)", met.SM, "-"));
    o.rows.push_back(si_row("V_tan (8.21)", met.V_tan, units::Dim::Speed));
    if (met.V_ch) o.rows.push_back(si_row("V_ch (8.19)", *met.V_ch, units::Dim::Speed,
                                          "understeering"));
    if (met.V_crit) o.rows.push_back(si_row("V_crit (8.20)", *met.V_crit, units::Dim::Speed,
                                            "OVERSTEERING — divergent above this speed"));
    o.rows.push_back(raw_row("omega_n", eig.omega_n, "rad/s"));
    o.rows.push_back(raw_row("zeta", eig.zeta, "-"));
    o.rows.push_back(raw_row("tau dominant", eig.tau_dominant, "s"));

    const auto ss = steady_state(veh, C, V, delta);
    if (!ss.ok()) return detail::fail(ss.error());
    o.rows.push_back(si_row("beta", ss.value().beta_rad, units::Dim::Angle));
    o.rows.push_back(rate_row("yaw rate r", ss.value().r_rad_per_s, d));
    o.rows.push_back(si_row("A_Y", ss.value().A_Y_mps2, units::Dim::Acceleration));
    o.rows.push_back(raw_row("rcond", ss.value().rcond, "-", "§13.8 reports it with every solve"));

    // (8.15) against (8.25). Plotting both is identity I-2 made visible: the
    // curves must be indistinguishable, and a sign error in §8.3 separates them.
    Series closed = detail::make_series("r/delta, (8.15) stability factor", 0);
    Series derivs = detail::make_series("r/delta, (8.25) derivative form", 1);
    constexpr int kN = 121;
    for (int i = 0; i < kN; ++i) {
        const double v_si = (i + 1) * (2.0 * V) / kN;
        const double x = *io::to_display(v_si, units::Dim::Speed, d);
        closed.x.push_back(x);
        closed.y.push_back(yaw_rate_gain(veh, met.K, v_si));
        derivs.x.push_back(x);
        derivs.y.push_back(yaw_rate_gain_derivative_form(veh, C, v_si));
    }
    o.series.push_back(std::move(closed));
    o.series.push_back(std::move(derivs));
    o.x_name = "V";
    o.x_unit = QString::fromStdString(std::string(d.unit_for(units::Dim::Speed)));
    o.y_name = "yaw-rate gain r/delta";
    o.y_unit = "1/s";
    o.annotation = "Both curves are the same quantity by identity I-2; any visible gap "
                   "is a §8.3 sign error.";
    for (const auto& w : veh.warnings) o.diagnostics.push_back(w);
    return o;
}

// -------------------------------------------------------------------------- P2
inline RunOutput run_p2(const core::Vehicle& veh, const Inputs& in, const io::DisplayUnits& d) {
    using namespace models;
    RunOutput o;
    const double V = detail::in_or(in, "speed");
    const double delta = detail::in_or(in, "steer");
    const double t_end = detail::in_or(in, "duration", 4.0);
    const double dt = detail::in_or(in, "dt", 1e-3);

    tire::LinearTireConfig tc;
    tc.id = "linear_breakaway";
    tc.C_alpha_N_per_rad = detail::in_or(in, "cf") / 2.0;   // per TIRE, two per axle
    tc.breakaway_slip_rad = detail::in_or(in, "breakaway");
    const auto front = tire::LinearTire::create(tc);
    if (!front.ok()) return detail::fail(front.error());
    tc.C_alpha_N_per_rad = detail::in_or(in, "cr") / 2.0;
    const auto rear = tire::LinearTire::create(tc);
    if (!rear.ok()) return detail::fail(rear.error());

    const std::array<const tire::TireModel*, 4> tires{&front.value(), &front.value(),
                                                      &rear.value(), &rear.value()};
    aero::AeroConfig no_aero;
    SolverSettings cfg;
    cfg.implicit_load_transfer = detail::in_or(in, "implicit") != 0.0;
    FourWheeler p2(veh, tires, no_aero, cfg);
    p2.set_speed(V);

    const Maneuver man = Maneuver::make_step(delta, 0.5);
    std::vector<double> x(state::kDim, 0.0), xd(state::kDim, 0.0), work(4 * state::kDim, 0.0);
    const numerics::Derivative f = [&](double t, std::span<const double> s,
                                      std::span<double> out) {
        p2.derivative(t, s, man.evaluate(t), out);
    };

    // §16.4 requires an axis label to carry its unit. A_Y (a fraction of g) and a
    // wheel load (thousands of newtons) cannot share one axis without the label
    // degenerating to "g / N" — and on a 0..6 kN scale A_Y is a flat line at the
    // bottom, which is worse than not plotting it. The four loads share one
    // dimension, so they get the frame; A_Y is in the results table.
    std::array<Series, 4> fz{detail::make_series("Fz LF", 0), detail::make_series("Fz RF", 1),
                             detail::make_series("Fz LR", 2), detail::make_series("Fz RR", 3)};
    const int steps = static_cast<int>(t_end / dt);
    FourWheelerObservation obs;
    for (int i = 0; i <= steps; ++i) {
        const double t = i * dt;
        p2.evaluate(t, x, man.evaluate(t), obs, xd);
        for (const auto& e : obs.events) o.diagnostics.push_back(e);
        if (!obs.events.empty() && obs.events.front().code.rfind("E-", 0) == 0) {
            return detail::fail(obs.events.front());
        }
        // One sample every 10 ms keeps the plot legible without changing the step.
        if (i % 10 == 0) {
            for (int w = 0; w < 4; ++w) {
                fz[static_cast<std::size_t>(w)].x.push_back(t);
                fz[static_cast<std::size_t>(w)].y.push_back(
                    *io::to_display(obs.Fz[static_cast<std::size_t>(w)], units::Dim::Force, d));
            }
        }
        if (i < steps) {
            numerics::rk4_step(f, t, x, dt, xd, work);
            std::copy(xd.begin(), xd.end(), x.begin());
        }
    }

    o.rows.push_back(si_row("A_Y at t_end", obs.A_Y, units::Dim::Acceleration));
    o.rows.push_back(si_row("beta at t_end", obs.beta, units::Dim::Angle));
    o.rows.push_back(rate_row("yaw rate at t_end", obs.r, d));
    o.rows.push_back(si_row("roll angle", obs.roll_angle, units::Dim::Angle));
    static constexpr std::array<const char*, 4> kCorner{"LF", "RF", "LR", "RR"};
    for (int w = 0; w < 4; ++w) {
        o.rows.push_back(si_row(std::string("Fz ") + kCorner[static_cast<std::size_t>(w)],
                                obs.Fz[static_cast<std::size_t>(w)], units::Dim::Force));
    }
    o.rows.push_back(raw_row("front lateral load transfer", obs.flt_front, "-"));
    o.rows.push_back(raw_row("rear lateral load transfer", obs.flt_rear, "-"));
    // §9.8 names this the primary debugging channel: it is zero at steady state,
    // so a non-zero value at the end of a settled run is the first thing to read.
    o.rows.push_back(raw_row("yaw-moment residual", obs.yaw_moment_residual, "N*m",
                             "§9.8: zero at steady state"));
    o.rows.push_back(raw_row("Phi' observed", obs.phi_prime, "-", "§9.7 contraction factor"));
    o.rows.push_back(raw_row("A_Y loop iterations", obs.ay_iterations, "-"));
    // Without this row a reader cannot tell whether "0 iterations" means the loop
    // converged at once or was never run at all.
    o.rows.push_back(text_row("load-transfer loop (§9.7)",
                              cfg.implicit_load_transfer
                                  ? "implicit - A_Y solved to a fixed point"
                                  : "explicit (the source's behaviour) - no loop run"));

    for (auto& s : fz) o.series.push_back(std::move(s));
    o.x_name = "t";
    o.x_unit = "s";
    o.y_name = "vertical wheel load Fz";
    o.y_unit = QString::fromStdString(std::string(d.unit_for(units::Dim::Force)));
    o.annotation = "Step steer at t = 0.5 s. A_Y and the slip angles are in the results "
                   "table; mixing them into this frame would cost the axis its unit.";
    for (const auto& w : veh.warnings) o.diagnostics.push_back(w);
    return o;
}

// -------------------------------------------------------------------------- P3
inline RunOutput run_p3(const core::Vehicle& veh, const Inputs& in, const io::DisplayUnits& d) {
    RunOutput o;
    tire::LinearTireConfig tc;
    tc.id = "linear_breakaway";
    tc.C_alpha_N_per_rad = detail::in_or(in, "cf") / 2.0;
    tc.breakaway_slip_rad = detail::in_or(in, "breakaway");
    const auto front = tire::LinearTire::create(tc);
    if (!front.ok()) return detail::fail(front.error());
    tc.C_alpha_N_per_rad = detail::in_or(in, "cr") / 2.0;
    const auto rear = tire::LinearTire::create(tc);
    if (!rear.ok()) return detail::fail(rear.error());

    analysis::MmmSweep sweep;
    sweep.V_mps = detail::in_or(in, "speed");
    const double radius = detail::in_or(in, "radius");
    sweep.radius_m = radius > 0.0 ? radius : std::numeric_limits<double>::infinity();
    sweep.beta_max = detail::in_or(in, "beta_max", 15.0 * units::DEGREE);
    sweep.beta_min = -sweep.beta_max;
    sweep.delta_max = detail::in_or(in, "delta_max", 15.0 * units::DEGREE);
    sweep.delta_min = -sweep.delta_max;

    const analysis::MomentMethod mmm(veh, front.value(), rear.value(), sweep);
    const auto met = analysis::metrics(mmm);
    const double abk = detail::in_or(in, "breakaway");
    const auto lim = analysis::construction_limits(veh, detail::in_or(in, "cf"), abk,
                                                   detail::in_or(in, "cr"), abk);

    o.rows.push_back(raw_row("stability index (10.6)", met.stability_index, "-"));
    // (10.7) is a free function taking the AXLE stiffnesses; metrics() cannot
    // recover them from a TireModel, so it is called here with the values the user
    // actually supplied.
    o.rows.push_back(raw_row("stability index, linear (10.7)",
                             analysis::stability_index_linear(veh, detail::in_or(in, "cf"),
                                                              detail::in_or(in, "cr")),
                             "-", "compare with (10.6) in the linear range"));
    o.rows.push_back(raw_row("control moment", met.control_moment, "1/rad"));
    o.rows.push_back(raw_row("max trimmed A_Y", met.max_trimmed_A_Y_bar, "g",
                             "§10.5 magnitude"));
    o.rows.push_back(si_row("at beta", met.max_trimmed_beta, units::Dim::Angle));
    o.rows.push_back(si_row("at delta", met.max_trimmed_delta, units::Dim::Angle));
    o.rows.push_back(raw_row("limit A_Y", met.limit_A_Y_bar, "g"));
    o.rows.push_back(raw_row("limit C_N", met.limit_C_N, "-"));
    o.rows.push_back(raw_row("trim-to-limit distance", met.trim_to_limit_distance, "-"));

    // Rule D2 forbids silent extrapolation, and the same spirit governs an
    // extremum reported from a sweep. Two distinct things can go wrong, and they
    // want opposite responses, so they are reported separately.
    //
    // (1) The maximum can be FLAT. A saturating tire model holds the front axle
    //     force constant past breakaway, the trim condition then fixes beta
    //     independently of delta, and every steer angle past breakaway attains
    //     the same maximum. Widening the sweep does NOT help — the plateau runs
    //     to the edge of any window — so the metric reports the least steer that
    //     reaches it, and the span is shown here.
    if (met.max_trimmed_is_plateau) {
        o.rows.push_back(raw_row("max trimmed, plateau from",
                                 *units::from_si(met.max_trimmed_delta_lo, "deg",
                                                 units::Dim::Angle),
                                 "deg", "the maximum is flat in steer, not a single point"));
        o.rows.push_back(raw_row("max trimmed, plateau to",
                                 *units::from_si(met.max_trimmed_delta_hi, "deg",
                                                 units::Dim::Angle),
                                 "deg", "widening the sweep only extends this span"));
    }
    // (2) The trim line can be CUT SHORT, when a steer station has no trim
    //     solution inside the beta window. Here widening — the beta sweep — is
    //     exactly the fix. The truncated end need not sit on the beta boundary,
    //     so this is reported from the station count rather than from the
    //     reported point's position.
    if (met.trim_line_truncated()) {
        o.rows.push_back(text_row(
            "trim line truncated",
            std::to_string(met.trim_points) + " of " +
                std::to_string(met.trim_points_requested) + " steer stations",
            "the rest have no trim solution inside the sideslip sweep — widen the "
            "sideslip sweep, or the maximum reported is the beta window's"));
    }
    o.rows.push_back(text_row("limit character (§10.5)", analysis::to_string(met.character)));
    o.rows.push_back(raw_row("front construction limit", lim.front_A_Y_bar, "g"));
    o.rows.push_back(raw_row("rear construction limit", lim.rear_A_Y_bar, "g"));

    // §10.3's C_N-A_Y_bar diagram: constant-beta lines against constant-delta
    // lines. grid() walks beta outermost, so a constant-beta line is contiguous.
    const auto pts = mmm.grid();
    const int nb = sweep.beta_n, nd = sweep.delta_n;
    const int stride = nb / 4;
    int style = 0;
    for (int i = 0; i < nb; i += stride) {
        Series s = detail::make_series(
            QString("beta = %1 deg")
                .arg(*units::from_si(pts[static_cast<std::size_t>(i) * nd].beta_rad, "deg",
                                     units::Dim::Angle),
                     0, 'f', 1),
            style++);
        for (int j = 0; j < nd; ++j) {
            const auto& p = pts[static_cast<std::size_t>(i) * nd + j];
            s.x.push_back(p.A_Y_bar);
            s.y.push_back(p.C_N);
        }
        o.series.push_back(std::move(s));
    }
    o.x_name = "A_Y / g (10.3a)";
    o.x_unit = "-";
    o.y_name = "C_N = N / (W l) (10.3b)";
    o.y_unit = "-";
    o.annotation = QString("%1 x %2 grid; constant-beta lines shown.").arg(nb).arg(nd);
    for (const auto& w : veh.warnings) o.diagnostics.push_back(w);
    (void)d;
    return o;
}

// ---------------------------------------------------------------------- P4 / P5
inline RunOutput run_speedway(const core::Vehicle& veh, const Inputs& in,
                              const io::DisplayUnits& d, bool four_wheeler) {
    using namespace models;
    RunOutput o;
    const AxleStiffness C{detail::in_or(in, "cf"), detail::in_or(in, "cr")};
    const double V = detail::in_or(in, "speed");
    const double delta = detail::in_or(in, "steer");

    analysis::Circuit circuit;
    circuit.inner_radius = detail::in_or(in, "inner");
    const double outer = detail::in_or(in, "outer");
    circuit.outer_radius = outer > circuit.inner_radius
                               ? outer
                               : std::numeric_limits<double>::infinity();

    analysis::DriverPlan plan;
    plan.initial_speed_mps = V;
    plan.steer = Maneuver::make_fixed(delta);
    const double start = detail::in_or(in, "start");
    plan.start_x = start > 0.0 ? start : circuit.inner_radius * 1.5;
    plan.start_heading_rad = units::PI / 2.0;

    const auto met = handling_metrics(veh, C);
    analysis::TransientContext ctx;
    ctx.tau_dominant_s = eigenstructure(veh, C, V).tau_dominant;
    ctx.steady_state_radius_m = delta != 0.0
                                    ? V / (yaw_rate_gain(veh, met.K, V) * delta)
                                    : std::numeric_limits<double>::infinity();
    analysis::LapConfig cfg;
    cfg.calculation_length_s = detail::in_or(in, "window", 120.0);

    // P4 is P1 on the circuit, P5 is P2 on the same circuit (§11). Both plants are
    // held for the whole solve, so the tire models must outlive it too.
    tire::LinearTireConfig tc;
    tc.id = "linear_breakaway";
    tc.breakaway_slip_rad = detail::in_or(in, "breakaway", 5.0 * units::DEGREE);
    tc.C_alpha_N_per_rad = C.C_F_N_per_rad / 2.0;
    const auto front = tire::LinearTire::create(tc);
    tc.C_alpha_N_per_rad = C.C_R_N_per_rad / 2.0;
    const auto rear = tire::LinearTire::create(tc);
    if (four_wheeler && !front.ok()) return detail::fail(front.error());
    if (four_wheeler && !rear.ok()) return detail::fail(rear.error());

    std::array<const tire::TireModel*, 4> tires{};
    aero::AeroConfig no_aero;
    SolverSettings settings;
    settings.implicit_load_transfer = detail::in_or(in, "implicit") != 0.0;
    BicyclePlant p1(veh, C, V);
    std::optional<FourWheeler> p2;
    if (four_wheeler) {
        tires = {&front.value(), &front.value(), &rear.value(), &rear.value()};
        p2.emplace(veh, tires, no_aero, settings);
        p2->set_speed(V);
    }
    const Plant& plant = four_wheeler ? static_cast<const Plant&>(*p2)
                                      : static_cast<const Plant&>(p1);

    const auto res = analysis::lap_solve(plant, circuit, plan, cfg, ctx);
    if (!res.ok()) return detail::fail(res.error());

    o.rows.push_back(text_row("termination event", analysis::to_string(res.value().event)));
    o.rows.push_back(raw_row("lap time", res.value().time_s, "s"));
    o.rows.push_back(raw_row("accepted steps", res.value().stats.accepted, "-"));
    o.rows.push_back(raw_row("rejected steps", res.value().stats.rejected, "-"));
    o.rows.push_back(si_row("steady-state radius", ctx.steady_state_radius_m,
                            units::Dim::Length, "from the (8.15) gain"));
    o.rows.push_back(raw_row("tau dominant", ctx.tau_dominant_s, "s"));
    for (const auto& dg : res.value().diagnostics) o.diagnostics.push_back(dg);

    Series path = detail::make_series("path", 0);
    for (const auto& s : res.value().path) {
        // §8.7: Y_E horizontal, X_E vertical.
        path.x.push_back(*io::to_display(s.Y_E, units::Dim::Length, d));
        path.y.push_back(*io::to_display(s.X_E, units::Dim::Length, d));
    }
    path.style.marker = SeriesStyle::none;   // a dense trajectory, not sampled data
    o.series.push_back(std::move(path));
    detail::set_path_axes(o, d);
    for (const auto& w : veh.warnings) o.diagnostics.push_back(w);
    return o;
}

// -------------------------------------------------------------------------- P6
inline RunOutput run_p6(const core::Vehicle& veh, const Inputs& in, const io::DisplayUnits& d) {
    RunOutput o;
    const auto axle = [&](const char* side, double track, double weight, const char* kw,
                          const char* ka, const char* kt) -> std::optional<core::Diagnostic> {
        analysis::AxleSpringing s;
        s.wheel_track = track;
        s.spring_track = track;
        s.axle_weight_N = weight;
        s.wheel_centre_rate_left = detail::in_or(in, kw);
        s.wheel_centre_rate_right = detail::in_or(in, kw);
        s.aux_roll_stiffness = detail::in_or(in, ka);
        const double tire_rate = detail::in_or(in, kt);
        s.tire_rate_per_tire =
            tire_rate > 0.0 ? tire_rate : std::numeric_limits<double>::infinity();

        const auto t = analysis::components_to_totals(s, veh.g);
        if (!t.ok()) return t.error();
        const std::string p(side);
        o.rows.push_back(si_row(p + " ride rate (12.1)", t.value().ride_rate,
                                units::Dim::Stiffness, "chassis to ground, per wheel"));
        o.rows.push_back(si_row(p + " roll stiffness, springs", t.value().roll_stiffness_springs,
                                units::Dim::AngularStiffness));
        o.rows.push_back(si_row(p + " roll stiffness, chassis to WC",
                                t.value().roll_stiffness_chassis_to_wc,
                                units::Dim::AngularStiffness));
        o.rows.push_back(si_row(p + " roll stiffness, TOTAL", t.value().roll_stiffness_total,
                                units::Dim::AngularStiffness,
                                "chassis to ground — the series tire step included"));
        o.rows.push_back(raw_row(p + " ride frequency", t.value().ride_frequency_Hz, "Hz"));
        for (const auto& w : t.value().warnings) o.diagnostics.push_back(w);
        return std::nullopt;
    };

    if (auto e = axle("front", veh.t_F, veh.W_F(), "kw_front", "arb_front", "tire_rate")) {
        return detail::fail(*e);
    }
    if (auto e = axle("rear", veh.t_R, veh.W_R(), "kw_rear", "arb_rear", "tire_rate")) {
        return detail::fail(*e);
    }

    // (12.11) against (12.12): the shortcut drops exactly m_s g h2, and showing
    // the two side by side is the only way a user sees how much that is.
    const double target = detail::in_or(in, "target_gradient", 1.5 * units::DEGREE);
    const auto tgt = analysis::roll_stiffness_for_gradient(
        veh.m_s, veh.h2(), target / veh.g, veh.g);
    o.rows.push_back(si_row("Sum K_phi for target, exact (12.11)", tgt.exact,
                            units::Dim::AngularStiffness));
    o.rows.push_back(si_row("Sum K_phi for target, simplified (12.12)", tgt.simplified,
                            units::Dim::AngularStiffness));
    o.rows.push_back(si_row("difference = m_s g h2", tgt.difference,
                            units::Dim::AngularStiffness,
                            "what the common shortcut silently omits"));

    // Sweep the front bar and show the resulting roll gradient. Each point is a
    // fresh core call, not an interpolation.
    Series exact = detail::make_series("exact (12.11)", 0);
    Series simple = detail::make_series("simplified (12.12)", 1);
    constexpr int kN = 61;
    for (int i = 0; i < kN; ++i) {
        const double grad_rad_per_mps2 = (i + 1) * (4.0 * units::DEGREE / veh.g) / kN;
        const auto t = analysis::roll_stiffness_for_gradient(veh.m_s, veh.h2(),
                                                            grad_rad_per_mps2, veh.g);
        const double x = *units::from_si(grad_rad_per_mps2 * veh.g, "deg", units::Dim::Angle);
        exact.x.push_back(x);
        exact.y.push_back(*io::to_display(t.exact, units::Dim::AngularStiffness, d));
        simple.x.push_back(x);
        simple.y.push_back(*io::to_display(t.simplified, units::Dim::AngularStiffness, d));
    }
    o.series.push_back(std::move(exact));
    o.series.push_back(std::move(simple));
    o.x_name = "target roll gradient";
    o.x_unit = "deg/g";
    o.y_name = "required total roll stiffness";
    o.y_unit = QString::fromStdString(std::string(d.unit_for(units::Dim::AngularStiffness)));
    o.annotation = "The gap between the curves is m_s g h2 — the term (12.12) drops.";
    for (const auto& w : veh.warnings) o.diagnostics.push_back(w);
    return o;
}

// ------------------------------------------------------------------- the roster
inline std::vector<ProgramSpec> all_programs() {
    using namespace fields;
    std::vector<ProgramSpec> p;

    ProgramSpec p1;
    p1.id = "P1";
    p1.title = "Bicycle Model";
    p1.section = "§8";
    p1.cli_command = "derivs";
    p1.vehicle_fields = vehicle(/*roll_dof=*/false, /*springing=*/false,
                                /*roll_centres=*/false);
    p1.run_fields = {cf(), cr(), speed(), steer()};
    p1.run = [](const core::Vehicle& v, const Inputs& i, const io::DisplayUnits& d) {
        return run_p1(v, i, d);
    };
    p.push_back(std::move(p1));

    ProgramSpec p2;
    p2.id = "P2";
    p2.title = "Four-Wheeler";
    p2.section = "§9";
    p2.vehicle_fields = vehicle(true, true, true);
    p2.run_fields = {cf(), cr(), breakaway(), speed(), steer(), implicit_loop(),
                     {"duration", "Run length", units::Dim::Time, Block::run, 4.0, false, false, false, {}, "s"},
                     {"dt", "Fixed step dt", units::Dim::Time, Block::run, 1e-3, false, false, false, {}, "s"}};
    p2.run = [](const core::Vehicle& v, const Inputs& i, const io::DisplayUnits& d) {
        return run_p2(v, i, d);
    };
    p.push_back(std::move(p2));

    ProgramSpec p3;
    p3.id = "P3";
    p3.title = "Moment Method (MMM)";
    p3.section = "§10";
    p3.vehicle_fields = vehicle(false, false, false);
    p3.run_fields = {
        cf(), cr(), breakaway(), speed(),
        {"radius", "Path radius (0 = straight)", units::Dim::Length, Block::run, 0.0, false,
         false, false, {}, "ft", true},
        {"beta_max", "Sideslip sweep +/-", units::Dim::Angle, Block::run, 15.0 * units::DEGREE,
         false, false, false, {}, "deg"},
        {"delta_max", "Steer sweep +/-", units::Dim::Angle, Block::run, 15.0 * units::DEGREE,
         false, false, false, {}, "deg"}};
    p3.run = [](const core::Vehicle& v, const Inputs& i, const io::DisplayUnits& d) {
        return run_p3(v, i, d);
    };
    p.push_back(std::move(p3));

    auto sentinel = [](FieldSpec f) {
        f.zero_means_default = true;
        return f;
    };
    const std::vector<FieldSpec> circuit = {
        {"inner", "Inner radius", units::Dim::Length, Block::run, 50.0 * units::FOOT, false,
         false, false, {}, "ft"},
        sentinel({"outer", "Outer radius (0 = none)", units::Dim::Length, Block::run, 0.0,
                  false, false, false, {}, "ft"}),
        sentinel({"start", "Start radius (0 = 1.5 x inner)", units::Dim::Length, Block::run, 0.0,
                  false, false, false, {}, "ft"})};

    ProgramSpec p4;
    p4.id = "P4";
    p4.title = "RCVD Speedway";
    p4.section = "§11";
    p4.cli_command = "lap";
    p4.vehicle_fields = vehicle(false, false, false);
    p4.run_fields = {cf(), cr(), speed(), steer()};
    p4.run_fields.insert(p4.run_fields.end(), circuit.begin(), circuit.end());
    p4.run = [](const core::Vehicle& v, const Inputs& i, const io::DisplayUnits& d) {
        return run_speedway(v, i, d, /*four_wheeler=*/false);
    };
    p.push_back(std::move(p4));

    ProgramSpec p5;
    p5.id = "P5";
    p5.title = "RCVD Speedway 2";
    p5.section = "§11";
    p5.vehicle_fields = vehicle(true, true, true);
    p5.run_fields = {cf(), cr(), breakaway(), speed(), steer(), implicit_loop()};
    p5.run_fields.insert(p5.run_fields.end(), circuit.begin(), circuit.end());
    p5.run = [](const core::Vehicle& v, const Inputs& i, const io::DisplayUnits& d) {
        return run_speedway(v, i, d, /*four_wheeler=*/true);
    };
    p.push_back(std::move(p5));

    ProgramSpec p6;
    p6.id = "P6";
    p6.title = "Ride and Roll Rate Calculator";
    p6.section = "§12";
    p6.vehicle_fields = vehicle(true, false, true);
    p6.run_fields = {
        {"kw_front", "Front wheel-centre rate k_W", units::Dim::Stiffness, Block::run,
         250.0 * units::LBF_PER_IN_TO_N_PER_M, false, false, false, {}, "lbf/in"},
        {"kw_rear", "Rear wheel-centre rate k_W", units::Dim::Stiffness, Block::run,
         200.0 * units::LBF_PER_IN_TO_N_PER_M, false, false, false, {}, "lbf/in"},
        {"arb_front", "Front bar roll stiffness", units::Dim::AngularStiffness, Block::run,
         300.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD, false, false, false, {}, "lbf*ft/deg"},
        {"arb_rear", "Rear bar roll stiffness", units::Dim::AngularStiffness, Block::run,
         150.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD, false, false, false, {}, "lbf*ft/deg"},
        {"tire_rate", "Tire rate per tire (0 = rigid)", units::Dim::Stiffness, Block::run,
         1500.0 * units::LBF_PER_IN_TO_N_PER_M, false, false, false, {}, "lbf/in"},
        {"target_gradient", "Target roll gradient", units::Dim::Angle, Block::run,
         1.5 * units::DEGREE, false, false, false, {}, "deg"}};
    p6.run = [](const core::Vehicle& v, const Inputs& i, const io::DisplayUnits& d) {
        return run_p6(v, i, d);
    };
    p.push_back(std::move(p6));

    return p;
}

}  // namespace rcvd::gui

#endif  // RCVD_GUI_PROGRAM_SPEC_HPP
