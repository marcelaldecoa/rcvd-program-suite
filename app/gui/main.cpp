// app/gui/main.cpp — the GUI entry point, spec §4.1 and §16.
#include <QApplication>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "launcher.hpp"

namespace {

// §18.3: "Any behaviour reachable through the GUI SHALL be reachable through
// rcvd-cli and through the C API with identical results, bit-for-bit." A claim
// like that is worth nothing if the only way to check it is to read numbers off a
// screenshot, so the GUI can print its own results at full precision with no
// window ever shown.
//
// It runs the SAME code the Run button runs: gui::vehicle_document ->
// io::load_vehicle -> core::resolve -> ProgramSpec::run. The only thing it
// substitutes is where the field values come from (their defaults instead of the
// widgets), so a difference against rcvd-cli is a real difference, not an artefact
// of a second implementation.
int selftest(const char* only) {
    using namespace rcvd;
    int failures = 0;
    for (const gui::ProgramSpec& p : gui::all_programs()) {
        if (only && p.id != only) continue;

        const auto loaded = io::load_vehicle(gui::vehicle_document(p, gui::default_value));
        if (!loaded.ok()) {
            std::printf("%s LOAD %s: %s\n", p.id.c_str(),
                        std::string(loaded.error().code).c_str(), loaded.error().message.c_str());
            ++failures;
            continue;
        }
        const auto veh = core::resolve(loaded.value());
        if (!veh.ok()) {
            std::printf("%s RESOLVE %s: %s\n", p.id.c_str(), std::string(veh.error().code).c_str(),
                        veh.error().message.c_str());
            ++failures;
            continue;
        }

        gui::Inputs in;
        for (const gui::FieldSpec& f : p.run_fields) in[f.key] = f.default_si;

        // SI display: the selftest compares physical values, so it must not route
        // them through a second conversion that the CLI would not apply.
        const io::DisplayUnits disp = io::DisplayUnits::of(io::UnitSystem::si);
        const gui::RunOutput out = p.run(veh.value(), in, disp);
        if (out.failure) {
            std::printf("%s RUN %s: %s\n", p.id.c_str(), std::string(out.failure->code).c_str(),
                        out.failure->message.c_str());
            ++failures;
            continue;
        }

        // %.17g round-trips a binary64 exactly, which is what "bit-for-bit" needs.
        //
        // The conversion below is the same one the Results table applies, and it
        // must be applied here too: §3.4 reports acceleration in g in BOTH unit
        // systems, so printing the SI number under the display unit's label would
        // mislabel every A_Y by a factor of g.
        for (const gui::ResultRow& row : out.rows) {
            if (!row.text.empty()) {
                std::printf("%s\t%s\t%s\t-\n", p.id.c_str(), row.name.c_str(), row.text.c_str());
                continue;
            }
            double shown = row.value;
            std::string unit = row.raw_unit;
            if (unit.empty()) {
                shown = *io::to_display(row.value, row.dim, disp);
                unit = std::string(disp.unit_for(row.dim));
            }
            std::printf("%s\t%s\t%.17g\t%s\n", p.id.c_str(), row.name.c_str(), shown,
                        unit.c_str());
        }
        for (const core::Diagnostic& d : out.diagnostics) {
            std::printf("%s\tDIAG\t%s\t%s\n", p.id.c_str(), std::string(d.code).c_str(),
                        d.message.c_str());
        }
        std::printf("%s\tSERIES\t%zu\t curves\n", p.id.c_str(), out.series.size());
    }
    return failures == 0 ? 0 : 1;
}

// The two halves of the §18.3 comparison, so it can be run from a script with no
// number ever retyped: the vehicle document the GUI would save, and the command
// line the Run tab would display for it.
// Render a window to a PNG with no display attached, so the §16.4 requirements
// that only the eye can check -- line style AND marker AND colour, axis labels
// carrying units, contrast -- can be reviewed and kept under review.
int render(const char* id, const char* path, int tab) {
    using namespace rcvd;
    for (const gui::ProgramSpec& p : gui::all_programs()) {
        if (p.id != id) continue;
        gui::ProgramWindow w(p);
        w.resize(1280, 820);
        w.setAttribute(Qt::WA_DontShowOnScreen, true);
        w.show();   // lays out and realises the widget; never reaches the display
        w.run_now();
        w.show_tab(tab);
        // Let layout and the deferred paint settle before grabbing.
        QCoreApplication::processEvents();
        const QPixmap shot = w.grab();
        if (!shot.save(QString::fromUtf8(path), "PNG")) {
            std::fprintf(stderr, "could not write %s\n", path);
            return 1;
        }
        std::printf("wrote %s (%dx%d)\n", path, shot.width(), shot.height());
        return 0;
    }
    std::fprintf(stderr, "no such program: %s\n", id);
    return 1;
}

int dump_vehicle(const char* id) {
    using namespace rcvd;
    for (const gui::ProgramSpec& p : gui::all_programs()) {
        if (p.id != id) continue;
        std::printf("%s\n", gui::vehicle_document(p, gui::default_value).dump().c_str());
        return 0;
    }
    return 1;
}

int dump_cli(const char* id, const char* vehicle_path) {
    using namespace rcvd;
    for (const gui::ProgramSpec& p : gui::all_programs()) {
        if (p.id != id) continue;
        if (p.cli_command.empty()) return 1;
        std::printf("%s --vehicle %s --in-units si --units si", p.cli_command.c_str(),
                    vehicle_path);
        for (const gui::FieldSpec& f : p.run_fields) {
            if (!gui::cli_exposes(p, f.key)) continue;
            if (f.zero_means_default && f.default_si == 0.0) continue;
            std::printf(" --%s %.17g", f.key.c_str(), f.default_si);
        }
        std::printf("\n");
        return 0;
    }
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    // The selftest needs QGuiApplication's type machinery (QString, QVector) but
    // no display, so it runs before any window is created and never opens one.
    if (argc > 3 && std::strcmp(argv[1], "--render") == 0) {
        // NOT the "offscreen" platform plugin: on Windows it comes up with an
        // empty font database and every glyph renders as a tofu box, which makes
        // the capture useless for checking §16.4's labelling. The normal platform
        // plugin plus WA_DontShowOnScreen (see render()) lays the window out and
        // rasterises it without it ever appearing.
        QApplication app(argc, argv);
        return render(argv[2], argv[3], argc > 4 ? std::atoi(argv[4]) : 2);
    }
    if (argc > 1 && std::strcmp(argv[1], "--selftest") == 0) {
        QCoreApplication app(argc, argv);
        return selftest(argc > 2 ? argv[2] : nullptr);
    }
    if (argc > 2 && std::strcmp(argv[1], "--dump-vehicle") == 0) {
        QCoreApplication app(argc, argv);
        return dump_vehicle(argv[2]);
    }
    if (argc > 3 && std::strcmp(argv[1], "--dump-cli") == 0) {
        QCoreApplication app(argc, argv);
        return dump_cli(argv[2], argv[3]);
    }

    QApplication app(argc, argv);
    app.setApplicationName("RCVD Program Suite");
    app.setOrganizationName("rcvd");

    rcvd::gui::Launcher shell;
    shell.show();

    // §16.1 allows a program to be opened directly: `rcvd-gui P3` skips the shell,
    // which is what a scripted demonstration needs.
    if (argc > 1) {
        const QString want = QString(argv[1]).toUpper();
        for (const rcvd::gui::ProgramSpec& p : rcvd::gui::all_programs()) {
            if (QString::fromStdString(p.id) == want) {
                auto* w = new rcvd::gui::ProgramWindow(p);
                w->setAttribute(Qt::WA_DeleteOnClose);
                w->show();
                break;
            }
        }
    }
    return app.exec();
}
