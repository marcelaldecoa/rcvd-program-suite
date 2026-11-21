// app/gui/unit_field.hpp — a numeric input that cannot be unit-ambiguous.
//
// §16.3: "Every numeric field carries an explicit unit selector; there are no
// implied units." That is enforced structurally here — the widget has no API that
// returns a bare number without a unit having been chosen, and value() returns SI
// because Rule U1 says the core only ever sees SI.
//
// §16.3 also requires that a field inert for the current model is DISABLED with a
// tooltip saying why, never merely ignored, and that validation is live, flagging
// the field with the same error code the core would raise.
#ifndef RCVD_GUI_UNIT_FIELD_HPP
#define RCVD_GUI_UNIT_FIELD_HPP

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>
#include <QWidget>

#include <optional>
#include <vector>

#include "rcvd/units/convert.hpp"

namespace rcvd::gui {

// The deuteranopia-safe palette of §16.4, also used for the field states so that
// "invalid" is never signalled by colour alone — the tooltip and the error code
// carry it too.
namespace palette {
inline constexpr const char* kError = "#B3261E";
inline constexpr const char* kErrorBg = "#FFF0EE";
inline constexpr const char* kDisabledNote = "#5F6368";
// Okabe-Ito vermillion-adjacent amber, dark enough on white for WCAG AA body text.
inline constexpr const char* kNote = "#8A5200";
}  // namespace palette

class UnitField : public QWidget {
    Q_OBJECT

public:
    UnitField(const QString& label, units::Dim dim, double initial_si, QWidget* parent = nullptr)
        : QWidget(parent), dim_(dim) {
        auto* row = new QHBoxLayout(this);
        row->setContentsMargins(0, 0, 0, 0);

        label_ = new QLabel(label, this);
        label_->setMinimumWidth(190);
        spin_ = new QDoubleSpinBox(this);
        spin_->setDecimals(6);
        spin_->setRange(-1e12, 1e12);
        spin_->setMinimumWidth(130);
        units_ = new QComboBox(this);

        // Only units of THIS dimension are offerable. §5's vocabulary is closed, so
        // the selector cannot present a unit the conversion layer would reject.
        for (const units::UnitEntry& e : units::kVocabulary) {
            if (e.dim != dim) continue;
            units_->addItem(QString::fromUtf8(e.name.data(), static_cast<int>(e.name.size())));
            offered_.emplace_back(e.name);
        }

        note_ = new QLabel(this);
        note_->setStyleSheet(QString("color:%1;").arg(palette::kError));
        note_->setVisible(false);

        row->addWidget(label_);
        row->addWidget(spin_);
        row->addWidget(units_);
        row->addWidget(note_, 1);

        set_si(initial_si);
        connect(spin_, &QDoubleSpinBox::valueChanged, this, [this] { emit changed(); });
        connect(units_, &QComboBox::currentIndexChanged, this, [this](int) {
            // Changing the unit RE-DISPLAYS the same physical quantity; it does not
            // reinterpret the number. Rule U2 allows one conversion here and the
            // stored value is SI, so the displayed number changes and the quantity
            // does not.
            set_si(si_);
            emit changed();
        });
    }

    // The only accessor. There is deliberately no raw_value().
    [[nodiscard]] double value_si() const {
        const auto si = units::to_si(spin_->value(), selected_unit(), dim_);
        return si.value_or(si_);
    }

    void set_si(double si) {
        si_ = si;
        const auto shown = units::from_si(si, selected_unit(), dim_);
        const QSignalBlocker block(spin_);
        spin_->setValue(shown.value_or(si));
    }

    void select_unit(std::string_view name) {
        for (int i = 0; i < units_->count(); ++i) {
            if (units_->itemText(i).toStdString() == name) {
                units_->setCurrentIndex(i);
                return;
            }
        }
    }

    [[nodiscard]] std::string_view selected_unit() const {
        const int i = units_->currentIndex();
        return (i >= 0 && i < static_cast<int>(offered_.size())) ? offered_[static_cast<std::size_t>(i)]
                                                                 : std::string_view{"-"};
    }

    // §16.3: live validation flags the field with the SAME error code the core
    // would raise, so the user sees E-TIRE-003 rather than "bad value".
    void set_error(std::string_view code, const QString& detail) {
        note_->setText(QString("%1 — %2").arg(QString::fromUtf8(code.data(), static_cast<int>(code.size())), detail));
        note_->setVisible(true);
        spin_->setStyleSheet(QString("background:%1; border:1px solid %2;")
                                 .arg(palette::kErrorBg, palette::kError));
        setToolTip(note_->text());
    }

    void clear_error() {
        note_->clear();
        note_->setVisible(false);
        note_->setStyleSheet(QString("color:%1;").arg(palette::kError));
        spin_->setStyleSheet(QString());
        setToolTip(QString());
    }

    // A constraint with NO §14.2 code behind it — a solver control such as a fixed
    // step, which the core never validates because it is not a document field.
    // Deliberately distinct from set_error: inventing a code for it would put a
    // string on screen that does not appear in the register, and §14.2 is a closed
    // register for exactly the reason that users look codes up.
    void set_note(const QString& text) {
        note_->setText(text);
        note_->setStyleSheet(QString("color:%1; font-style:italic;").arg(palette::kNote));
        note_->setVisible(true);
        setToolTip(text);
    }

    // §16.3: "Fields inert for the current model (e.g. CG height in P1) are
    // disabled with a tooltip saying why, never merely ignored."
    void set_inert(const QString& why) {
        spin_->setEnabled(false);
        units_->setEnabled(false);
        label_->setStyleSheet(QString("color:%1;").arg(palette::kDisabledNote));
        const QString msg = QString("Not used by this model: %1").arg(why);
        setToolTip(msg);
        spin_->setToolTip(msg);
        label_->setToolTip(msg);
        note_->setText(QString("inert — %1").arg(why));
        note_->setStyleSheet(QString("color:%1; font-style:italic;").arg(palette::kDisabledNote));
        note_->setVisible(true);
    }

    [[nodiscard]] bool is_inert() const { return !spin_->isEnabled(); }

signals:
    void changed();

private:
    units::Dim dim_;
    double si_ = 0.0;
    QLabel* label_;
    QLabel* note_;
    QDoubleSpinBox* spin_;
    QComboBox* units_;
    std::vector<std::string_view> offered_;
};

}  // namespace rcvd::gui

#endif  // RCVD_GUI_UNIT_FIELD_HPP
