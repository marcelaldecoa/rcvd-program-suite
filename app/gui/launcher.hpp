// app/gui/launcher.hpp — the common shell of §16.1.
//
// "Each program SHALL be a separate window, launched from a common shell." The
// shell owns nothing but the windows: it holds no vehicle, no units and no
// results, so two programs open at once cannot contaminate each other.
#ifndef RCVD_GUI_LAUNCHER_HPP
#define RCVD_GUI_LAUNCHER_HPP

#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <memory>
#include <vector>

#include "program_window.hpp"

namespace rcvd::gui {

class Launcher : public QWidget {
    Q_OBJECT

public:
    Launcher(QWidget* parent = nullptr) : QWidget(parent), programs_(all_programs()) {
        setWindowTitle("RCVD Program Suite");
        resize(560, 420);

        auto* col = new QVBoxLayout(this);
        auto* title = new QLabel("<b>Race Car Vehicle Dynamics Program Suite</b>", this);
        col->addWidget(title);
        auto* sub = new QLabel(
            "Six programs after Milliken, Kasprzak, Metz and Milliken, <i>Race Car "
            "Vehicle Dynamics: Problems, Answers and Experiments</i> (SAE R-280). "
            "Each opens in its own window; the core computes in SI and every field "
            "names its own unit.",
            this);
        sub->setWordWrap(true);
        col->addWidget(sub);

        list_ = new QListWidget(this);
        for (const ProgramSpec& p : programs_) {
            auto* item = new QListWidgetItem(
                QString("%1  —  %2      (%3)")
                    .arg(QString::fromStdString(p.id), QString::fromStdString(p.title),
                         QString::fromStdString(p.section)),
                list_);
            item->setToolTip(p.cli_command.empty()
                                 ? QString("no rcvd-cli subcommand; reachable through the C API")
                                 : QString("rcvd-cli %1")
                                       .arg(QString::fromStdString(p.cli_command)));
        }
        list_->setCurrentRow(0);
        col->addWidget(list_, 1);

        auto* open = new QPushButton("Open program", this);
        open->setDefault(true);
        col->addWidget(open);

        auto* foot = new QLabel(
            "Rule A1: everything reachable here is reachable through rcvd-cli and "
            "the C API with identical results. The Run tab of each window prints "
            "the equivalent command line.",
            this);
        foot->setWordWrap(true);
        col->addWidget(foot);

        connect(open, &QPushButton::clicked, this, [this] { open_current(); });
        connect(list_, &QListWidget::itemDoubleClicked, this,
                [this](QListWidgetItem*) { open_current(); });
    }

private:
    void open_current() {
        const int row = list_->currentRow();
        if (row < 0 || row >= static_cast<int>(programs_.size())) return;
        // A fresh window per activation, each with its own independent state.
        auto* w = new ProgramWindow(programs_[static_cast<std::size_t>(row)]);
        w->setAttribute(Qt::WA_DeleteOnClose);
        w->show();
    }

    std::vector<ProgramSpec> programs_;
    QListWidget* list_ = nullptr;
};

}  // namespace rcvd::gui

#endif  // RCVD_GUI_LAUNCHER_HPP
