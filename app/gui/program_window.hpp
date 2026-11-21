// app/gui/program_window.hpp — one window per program, spec §16.1-§16.4.
//
// §16.1: "Define / Run / Results / Diagnostics", in that order, for every program.
// Rule A1 keeps this file to layout, signal wiring and formatting; every number
// it shows came from a core call made in program_spec.hpp.
#ifndef RCVD_GUI_PROGRAM_WINDOW_HPP
#define RCVD_GUI_PROGRAM_WINDOW_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextStream>
#include <QVBoxLayout>
#include <QWidget>

#include <array>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "rcvd/io/document.hpp"
#include "rcvd/io/json.hpp"

#include "plot_widget.hpp"
#include "program_spec.hpp"
#include "unit_field.hpp"

namespace rcvd::gui {

class ProgramWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit ProgramWindow(ProgramSpec spec, QWidget* parent = nullptr)
        : QMainWindow(parent), spec_(std::move(spec)) {
        setWindowTitle(QString("%1 — %2  (%3)")
                           .arg(QString::fromStdString(spec_.id),
                                QString::fromStdString(spec_.title),
                                QString::fromStdString(spec_.section)));
        resize(1180, 760);

        tabs_ = new QTabWidget(this);
        tabs_->addTab(build_define_tab(), "Define");
        tabs_->addTab(build_run_tab(), "Run");
        tabs_->addTab(build_results_tab(), "Results");
        tabs_->addTab(build_diagnostics_tab(), "Diagnostics");
        setCentralWidget(tabs_);

        status_ = new QLabel("ready", this);
        statusBar()->addWidget(status_);

        validate_live();
        refresh_cli();
    }

    // Press Run programmatically. Used by --render so the §16.4 appearance of a
    // plot can be captured and reviewed without a person clicking anything.
    void run_now() { execute(); }

    // Show a given tab by index: 0 Define, 1 Run, 2 Results, 3 Diagnostics.
    void show_tab(int i) { tabs_->setCurrentIndex(i); }

private:
    // ------------------------------------------------------------- Define tab
    QWidget* build_define_tab() {
        auto* page = new QWidget(this);
        auto* outer = new QVBoxLayout(page);

        auto* note = new QLabel(
            "Every field carries its own unit selector (§16.3); the core computes "
            "in SI only (Rule U1) and each number crosses the boundary exactly "
            "twice (Rule U2). Greyed fields are inert for this program — hover to "
            "see why.",
            page);
        note->setWordWrap(true);
        outer->addWidget(note);

        auto* scroll = new QScrollArea(page);
        scroll->setWidgetResizable(true);
        auto* inner = new QWidget(scroll);
        auto* col = new QVBoxLayout(inner);

        static constexpr std::array<std::pair<Block, const char*>, 4> kGroups{
            {{Block::inertia, "Inertia (§5.1)"},
             {Block::geometry, "Geometry (§5.1)"},
             {Block::springing, "Springing — chassis to ground (§9.3 D-5)"},
             {Block::steering, "Steering"}}};

        for (const auto& [block, title] : kGroups) {
            QGroupBox* box = nullptr;
            QVBoxLayout* rows = nullptr;
            for (const FieldSpec& f : spec_.vehicle_fields) {
                if (f.block != block) continue;
                if (!box) {
                    box = new QGroupBox(title, inner);
                    rows = new QVBoxLayout(box);
                }
                rows->addWidget(add_field(f, box));
            }
            if (box) col->addWidget(box);
        }
        col->addStretch(1);
        scroll->setWidget(inner);
        outer->addWidget(scroll, 1);
        return page;
    }

    // ---------------------------------------------------------------- Run tab
    QWidget* build_run_tab() {
        auto* page = new QWidget(this);
        auto* outer = new QVBoxLayout(page);

        auto* box = new QGroupBox("Run parameters", page);
        auto* rows = new QVBoxLayout(box);
        for (const FieldSpec& f : spec_.run_fields) rows->addWidget(add_field(f, box));
        outer->addWidget(box);

        auto* bar = new QWidget(page);
        auto* bl = new QHBoxLayout(bar);
        bl->setContentsMargins(0, 0, 0, 0);
        bl->addWidget(new QLabel("Display units:", bar));
        display_ = new QComboBox(bar);
        display_->addItem("SI", QVariant::fromValue(0));
        display_->addItem("US customary", QVariant::fromValue(1));
        bl->addWidget(display_);
        auto* run = new QPushButton("Run", bar);
        run->setDefault(true);
        bl->addWidget(run);
        auto* save = new QPushButton("Save vehicle JSON...", bar);
        bl->addWidget(save);
        bl->addStretch(1);
        outer->addWidget(bar);

        // §16.2: "the window SHALL be able to display the equivalent CLI command
        // for the current configuration." Generated from the live field values, so
        // it cannot drift from what the window would compute.
        auto* cli_box = new QGroupBox("Equivalent CLI command (§16.2)", page);
        auto* cl = new QVBoxLayout(cli_box);
        cli_ = new QPlainTextEdit(cli_box);
        cli_->setReadOnly(true);
        cli_->setMaximumHeight(110);
        cli_->setFont(QFont("Consolas"));
        cl->addWidget(cli_);
        auto* why = new QLabel(
            "Emitted with --in-units si because that option selects a whole system, "
            "while the form above lets each field choose its own unit. The numbers "
            "are the same physical quantities, already in SI.",
            cli_box);
        why->setWordWrap(true);
        cl->addWidget(why);
        outer->addWidget(cli_box);

        failure_ = new QLabel(page);
        failure_->setWordWrap(true);
        failure_->setStyleSheet(QString("color:%1; font-weight:bold;").arg(palette::kError));
        failure_->setVisible(false);
        outer->addWidget(failure_);
        outer->addStretch(1);

        connect(run, &QPushButton::clicked, this, [this] { execute(); });
        connect(save, &QPushButton::clicked, this, [this] { save_vehicle(); });
        connect(display_, &QComboBox::currentIndexChanged, this, [this](int) {
            refresh_cli();
            if (ran_) execute();
        });
        return page;
    }

    QWidget* build_results_tab() {
        auto* split = new QSplitter(Qt::Horizontal, this);
        table_ = new QTableWidget(split);
        table_->setColumnCount(4);
        table_->setHorizontalHeaderLabels({"quantity", "value", "unit", "note"});
        table_->horizontalHeader()->setStretchLastSection(true);
        table_->verticalHeader()->setVisible(false);
        table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        plot_ = new PlotWidget(split);
        split->addWidget(table_);
        split->addWidget(plot_);
        split->setStretchFactor(0, 3);
        split->setStretchFactor(1, 4);
        return split;
    }

    QWidget* build_diagnostics_tab() {
        auto* page = new QWidget(this);
        auto* l = new QVBoxLayout(page);
        auto* note = new QLabel(
            "Every code here appears in the §14.2 register, and its severity is read "
            "from that register — a caller cannot downgrade an E- code to a warning. "
            "A W- diagnostic travels with a successful result and must still be read.",
            page);
        note->setWordWrap(true);
        l->addWidget(note);
        diag_ = new QTableWidget(page);
        diag_->setColumnCount(5);
        diag_->setHorizontalHeaderLabels({"code", "severity", "t [s]", "register meaning",
                                          "message"});
        diag_->horizontalHeader()->setStretchLastSection(true);
        diag_->verticalHeader()->setVisible(false);
        diag_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        l->addWidget(diag_, 1);
        return page;
    }

    QWidget* add_field(const FieldSpec& f, QWidget* parent) {
        if (f.is_switch) {
            // A model switch. No unit selector, because it has no dimension; §16.3
            // governs numeric fields and this is not one.
            auto* c = new QCheckBox(QString::fromStdString(f.label), parent);
            c->setChecked(f.switch_default);
            switches_[f.key] = c;
            specs_[f.key] = f;
            connect(c, &QCheckBox::toggled, this, [this](bool) { refresh_cli(); });
            return c;
        }
        auto* w = new UnitField(QString::fromStdString(f.label), f.dim, f.default_si, parent);
        if (!f.us_unit.empty()) w->select_unit(f.us_unit);
        if (!f.inert_reason.empty()) w->set_inert(QString::fromStdString(f.inert_reason));
        fields_[f.key] = w;
        specs_[f.key] = f;
        connect(w, &UnitField::changed, this, [this] {
            validate_live();
            refresh_cli();
        });
        return w;
    }

    // One accessor for both kinds, so the run functions never need to know which
    // widget a key came from. A switch reads back as 0.0 or 1.0.
    [[nodiscard]] double si(const std::string& key) const {
        const auto sw = switches_.find(key);
        if (sw != switches_.end()) return sw->second->isChecked() ? 1.0 : 0.0;
        const auto it = fields_.find(key);
        return it == fields_.end() ? 0.0 : it->second->value_si();
    }

    [[nodiscard]] io::DisplayUnits display() const {
        return io::DisplayUnits::of(display_->currentIndex() == 1 ? io::UnitSystem::us
                                                                  : io::UnitSystem::si);
    }

    // §16.3: live validation "flags the field with the same error code the core
    // would raise". These are the checks expressible without a solve; everything
    // else surfaces on Run, from the core, with the core's own code.
    void validate_live() {
        for (const auto& [key, w] : fields_) {
            if (w->is_inert()) continue;
            const FieldSpec& f = specs_.at(key);
            w->clear_error();
            const double v = w->value_si();
            if ((key == "cf" || key == "cr") && v >= 0.0) {
                w->set_error("E-TIRE-003",
                             "§3.3 fixes the sign convention: cornering stiffness is "
                             "negative, so a positive value is rejected, not corrected");
            } else if (key == "front_weight_fraction" && !(v > 0.0 && v < 1.0)) {
                w->set_error("E-VEH-010", "must lie in the open interval (0, 1)");
            } else if ((key == "dt" || key == "duration") && !(v > 0.0)) {
                // A solver control, not a §5.1 document field: the core raises no
                // code for it, so none is shown. See UnitField::set_note.
                w->set_note("must be positive — solver control, no §14.2 code");
            } else if (f.dim == units::Dim::Force && key == "weight" && !(v > 0.0)) {
                w->set_error("E-VEH-010", "weight must be positive");
            }
        }
    }

    // The §5.1 document for the current field values. The builder is shared with
    // the selftest path in program_spec.hpp, so what the window computes and what
    // the §18.3 cross-check computes come from one place.
    [[nodiscard]] io::Json vehicle_document() const {
        return gui::vehicle_document(spec_, [this](const FieldSpec& f) { return si(f.key); });
    }

    void refresh_cli() {
        QString cmd;
        if (spec_.cli_command.empty()) {
            cmd = QString("# %1 is not exposed as an rcvd-cli subcommand in this build.\n"
                          "# The C API covers it: see include/rcvd/capi/rcvd.h.")
                      .arg(QString::fromStdString(spec_.id));
        } else {
            cmd = QString("rcvd-cli %1 --vehicle %2 --in-units si --units %3")
                      .arg(QString::fromStdString(spec_.cli_command),
                           vehicle_path_.isEmpty() ? "<save the vehicle JSON first>"
                                                   : vehicle_path_,
                           display_->currentIndex() == 1 ? "us" : "si");
            for (const FieldSpec& f : spec_.run_fields) {
                if (!gui::cli_exposes(spec_, f.key)) continue;
                if (f.zero_means_default && si(f.key) == 0.0) continue;
                // ONE line, with no continuation character. A trailing backslash
                // is bash; PowerShell wants a backtick and cmd a caret, so any
                // single choice is wrong in two shells out of three — and this
                // project is driven from PowerShell. The box scrolls and the text
                // is selectable, so a long line costs nothing and always pastes.
                cmd += QString(" --%1 %2")
                           .arg(QString::fromStdString(f.key),
                                QString::number(si(f.key), 'g', 17));
            }
        }
        cli_->setLineWrapMode(QPlainTextEdit::NoWrap);
        cli_->setPlainText(cmd);
    }

    void save_vehicle() {
        const QString path = QFileDialog::getSaveFileName(
            this, "Save vehicle document", QString::fromStdString(spec_.id) + "_vehicle.json",
            "JSON (*.json)");
        if (path.isEmpty()) return;
        std::ofstream out(path.toStdString(), std::ios::binary);
        out << vehicle_document().dump();
        vehicle_path_ = path;
        refresh_cli();
        status_->setText("wrote " + path);
    }

    void show_failure(const core::Diagnostic& d) {
        failure_->setText(QString("%1 — %2")
                              .arg(QString::fromUtf8(d.code.data(),
                                                     static_cast<int>(d.code.size())),
                                   QString::fromStdString(d.message)));
        failure_->setVisible(true);
        fill_diagnostics({d});
        status_->setText("run rejected");
    }

    void execute() {
        failure_->setVisible(false);
        const io::DisplayUnits disp = display();

        // Input path, identical to the CLI's: document -> load_vehicle -> resolve.
        const auto loaded = io::load_vehicle(vehicle_document());
        if (!loaded.ok()) return show_failure(loaded.error());
        const auto resolved = core::resolve(loaded.value());
        if (!resolved.ok()) return show_failure(resolved.error());

        Inputs in;
        for (const FieldSpec& f : spec_.run_fields) in[f.key] = si(f.key);

        const RunOutput out = spec_.run(resolved.value(), in, disp);
        if (out.failure) return show_failure(*out.failure);

        fill_results(out, disp);
        fill_diagnostics(out.diagnostics);
        plot_->clear();
        plot_->set_axes(out.x_name, out.x_unit, out.y_name, out.y_unit);
        for (const Series& s : out.series) plot_->add(s);
        if (!out.annotation.isEmpty()) plot_->set_annotation(out.annotation);
        ran_ = true;
        status_->setText(QString("%1 rows, %2 diagnostics, %3 axes")
                             .arg(out.rows.size())
                             .arg(out.diagnostics.size())
                             .arg(plot_->square_axes() ? "square" : "independent"));
        tabs_->setCurrentIndex(2);
    }

    void fill_results(const RunOutput& out, const io::DisplayUnits& disp) {
        table_->setRowCount(static_cast<int>(out.rows.size()));
        int r = 0;
        for (const ResultRow& row : out.rows) {
            double shown = row.value;
            std::string unit = row.raw_unit;
            if (!row.text.empty()) {
                table_->setItem(r, 0, new QTableWidgetItem(QString::fromStdString(row.name)));
                table_->setItem(r, 1, new QTableWidgetItem(QString::fromStdString(row.text)));
                table_->setItem(r, 2, new QTableWidgetItem("-"));
                table_->setItem(r, 3, new QTableWidgetItem(QString::fromStdString(row.note)));
                ++r;
                continue;
            }
            if (unit.empty()) {
                // Rule U2's display conversion: exactly one, here, at the boundary.
                shown = *io::to_display(row.value, row.dim, disp);
                unit = std::string(disp.unit_for(row.dim));
            }
            table_->setItem(r, 0, new QTableWidgetItem(QString::fromStdString(row.name)));
            auto* v = new QTableWidgetItem(QString::number(shown, 'g', 10));
            v->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
            table_->setItem(r, 1, v);
            table_->setItem(r, 2, new QTableWidgetItem(QString::fromStdString(unit)));
            table_->setItem(r, 3, new QTableWidgetItem(QString::fromStdString(row.note)));
            ++r;
        }
        fit_columns(table_);
    }

    // resizeColumnsToContents() sizes to the WIDEST cell, so one long sentence in
    // a categorical row widens its column until the right-aligned numbers beside
    // it are pushed out of the viewport entirely — the table then looks empty.
    // Cap every column and keep the full text reachable as a tooltip.
    static void fit_columns(QTableWidget* t) {
        t->resizeColumnsToContents();
        constexpr int kMaxColumn = 280;
        for (int c = 0; c < t->columnCount(); ++c) {
            if (t->columnWidth(c) > kMaxColumn) t->setColumnWidth(c, kMaxColumn);
        }
        for (int r = 0; r < t->rowCount(); ++r) {
            for (int c = 0; c < t->columnCount(); ++c) {
                if (QTableWidgetItem* it = t->item(r, c)) it->setToolTip(it->text());
            }
        }
    }

    void fill_diagnostics(const std::vector<core::Diagnostic>& ds) {
        diag_->setRowCount(static_cast<int>(ds.size()));
        int r = 0;
        for (const core::Diagnostic& d : ds) {
            const QString code =
                QString::fromUtf8(d.code.data(), static_cast<int>(d.code.size()));
            diag_->setItem(r, 0, new QTableWidgetItem(code));
            diag_->setItem(r, 1, new QTableWidgetItem(severity_name(d.severity)));
            diag_->setItem(r, 2, new QTableWidgetItem(
                                     d.t_s ? QString::number(*d.t_s, 'g', 6) : QString("-")));
            const auto entry = core::codes::find(d.code);
            diag_->setItem(
                r, 3,
                new QTableWidgetItem(entry ? QString::fromUtf8(
                                                 entry->meaning.data(),
                                                 static_cast<int>(entry->meaning.size()))
                                           : QString("<not in the §14.2 register>")));
            diag_->setItem(r, 4, new QTableWidgetItem(QString::fromStdString(d.message)));
            // Severity is carried by the TEXT, not only by colour (§16.4).
            if (d.severity == core::Severity::Error) {
                diag_->item(r, 0)->setForeground(QColor(palette::kError));
            }
            ++r;
        }
        fit_columns(diag_);
    }

    [[nodiscard]] static QString severity_name(core::Severity s) {
        switch (s) {
            case core::Severity::Error:   return "Error";
            case core::Severity::Warning: return "Warning";
            case core::Severity::Event:   return "Event";
            case core::Severity::Hint:    return "Hint";
        }
        return "?";
    }

    ProgramSpec spec_;
    QTabWidget* tabs_ = nullptr;
    QComboBox* display_ = nullptr;
    QPlainTextEdit* cli_ = nullptr;
    QTableWidget* table_ = nullptr;
    QTableWidget* diag_ = nullptr;
    PlotWidget* plot_ = nullptr;
    QLabel* failure_ = nullptr;
    QLabel* status_ = nullptr;
    QString vehicle_path_;
    bool ran_ = false;
    std::map<std::string, UnitField*> fields_;
    std::map<std::string, QCheckBox*> switches_;
    std::map<std::string, FieldSpec> specs_;
};

}  // namespace rcvd::gui

#endif  // RCVD_GUI_PROGRAM_WINDOW_HPP
