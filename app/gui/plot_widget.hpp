// app/gui/plot_widget.hpp — the plot surface, spec §15.2, §16.4 and §8.7.
//
// Written with QPainter rather than QCustomPlot. §16 says "QCustomPlot or
// equivalent"; QCustomPlot is GPL for non-commercial use, which §0.4's care about
// licensing makes worth avoiding for a shipped product, and the widget below owes
// nothing to it.
//
// §16's own constraint on this file: "the plot widget consumes only Result value
// types". It therefore holds plain series of SI doubles plus their display units
// and knows nothing about vehicles, tires or solvers.
#ifndef RCVD_GUI_PLOT_WIDGET_HPP
#define RCVD_GUI_PLOT_WIDGET_HPP

#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QString>
#include <QVector>
#include <QWidget>

#include <algorithm>
#include <cmath>

namespace rcvd::gui {

// §16.4: "Curves SHALL be distinguishable by line style and marker as well as
// colour (deuteranopia-safe palette)." All three vary together, so the plot is
// readable in greyscale and to a deuteranope.
struct SeriesStyle {
    QColor colour;
    Qt::PenStyle line;
    enum Marker { none, circle, square, triangle, cross } marker;
};

inline QVector<SeriesStyle> default_styles() {
    // Okabe-Ito, which is deuteranopia- and protanopia-safe.
    return {
        {QColor("#0072B2"), Qt::SolidLine, SeriesStyle::circle},
        {QColor("#D55E00"), Qt::DashLine, SeriesStyle::square},
        {QColor("#009E73"), Qt::DotLine, SeriesStyle::triangle},
        {QColor("#CC79A7"), Qt::DashDotLine, SeriesStyle::cross},
        {QColor("#56B4E9"), Qt::DashDotDotLine, SeriesStyle::circle},
    };
}

struct Series {
    QString name;
    QVector<double> x, y;   // already in DISPLAY units — Rule U2 converts once, upstream
    SeriesStyle style;
};

class PlotWidget : public QWidget {
    Q_OBJECT

public:
    explicit PlotWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumSize(420, 320);
        setAutoFillBackground(true);
    }

    void clear() {
        series_.clear();
        annotation_.clear();
        update();
    }

    void add(Series s) {
        if (s.style.colour.isValid() == false) {
            const auto styles = default_styles();
            s.style = styles[static_cast<int>(series_.size()) % styles.size()];
        }
        series_.push_back(std::move(s));
        update();
    }

    // §16.4: "Axis labels SHALL always carry units." The unit is a separate
    // argument so it cannot be forgotten by writing it into the name.
    void set_axes(const QString& x_name, const QString& x_unit, const QString& y_name,
                  const QString& y_unit) {
        x_name_ = x_name;
        x_unit_ = x_unit;
        y_name_ = y_name;
        y_unit_ = y_unit;
        update();
    }

    // §15.2: "Square-axis mode SHALL be available and SHALL be the default for any
    // plot whose two axes carry the same unit." The caller does not decide that —
    // the widget does, from the units it was given.
    void set_square_axes(bool on) {
        square_ = on;
        square_explicit_ = true;
        update();
    }

    [[nodiscard]] bool square_axes() const {
        if (square_explicit_) return square_;
        // §15.2's default is for a plot "whose two axes carry the same unit" — a
        // path plot, where equal scaling is what makes a circle look circular.
        // Two DIMENSIONLESS axes are not that case: A_Y/g against C_N on an MMM
        // diagram share the placeholder "-" while measuring unrelated things, and
        // forcing equal scaling there distorts the diagram instead of honouring it.
        if (x_unit_.isEmpty() || x_unit_ == "-") return false;
        return x_unit_ == y_unit_;
    }

    // §8.7 requires the path plot to annotate itself when the axis convention is
    // not the default, because the non-default orientation MIRRORS the path and a
    // right-hand turn is then drawn as a left-hand turn.
    void set_annotation(const QString& text) {
        annotation_ = text;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // §16.4 WCAG AA: a near-white ground with near-black ink is ~18:1, and every
        // label below is drawn in the ink colour, never in a series colour.
        const QColor ground("#FFFFFF");
        const QColor ink("#1A1A1A");
        const QColor grid("#D0D0D0");
        p.fillRect(rect(), ground);

        const QFontMetrics fm(p.font());
        const int left = 78, bottom = 52, top = 16;
        const int right = 16 + legend_width(fm);
        QRect box(left, top, width() - left - right, height() - top - bottom);
        if (box.width() < 60 || box.height() < 60) return;

        p.setPen(QPen(ink, 1));
        p.drawRect(box);

        double x0 = 0, x1 = 1, y0 = 0, y1 = 1;
        if (!bounds(x0, x1, y0, y1)) {
            p.setPen(ink);
            p.drawText(box, Qt::AlignCenter, "no data — press Run");
            draw_axis_titles(p, ink, box);
            return;
        }

        if (square_axes()) {
            // Equal data-units-per-pixel on both axes, so a circular path renders
            // as a circle. Expand the narrower range rather than cropping.
            const double xs = (x1 - x0) / box.width();
            const double ys = (y1 - y0) / box.height();
            const double s = std::max(xs, ys);
            const double cx = 0.5 * (x0 + x1), cy = 0.5 * (y0 + y1);
            const double hw = 0.5 * s * box.width(), hh = 0.5 * s * box.height();
            x0 = cx - hw; x1 = cx + hw;
            y0 = cy - hh; y1 = cy + hh;
        }

        const auto px = [&](double v) {
            return box.left() + (v - x0) / (x1 - x0) * box.width();
        };
        const auto py = [&](double v) {
            return box.bottom() - (v - y0) / (y1 - y0) * box.height();
        };

        // Gridlines and tick labels. Every label carries its unit (§16.4).
        p.setPen(QPen(grid, 1, Qt::DotLine));
        constexpr int kTicks = 5;
        for (int i = 0; i <= kTicks; ++i) {
            const double fx = x0 + (x1 - x0) * i / kTicks;
            const double fy = y0 + (y1 - y0) * i / kTicks;
            p.setPen(QPen(grid, 1, Qt::DotLine));
            p.drawLine(QPointF(px(fx), box.top()), QPointF(px(fx), box.bottom()));
            p.drawLine(QPointF(box.left(), py(fy)), QPointF(box.right(), py(fy)));
            p.setPen(ink);
            p.drawText(QRectF(px(fx) - 46, box.bottom() + 4, 92, fm.height()),
                       Qt::AlignHCenter, tick(fx));
            p.drawText(QRectF(2, py(fy) - fm.height() / 2.0, left - 8, fm.height()),
                       Qt::AlignRight | Qt::AlignVCenter, tick(fy));
        }

        // Series: colour AND dash pattern AND marker, all three (§16.4).
        for (const Series& s : series_) {
            if (s.x.size() < 1) continue;
            p.setPen(QPen(s.style.colour, 2.0, s.style.line));
            QPainterPath path;
            for (int i = 0; i < s.x.size(); ++i) {
                const QPointF pt(px(s.x[i]), py(s.y[i]));
                if (i == 0) path.moveTo(pt); else path.lineTo(pt);
            }
            p.drawPath(path);

            // Markers at a stride, so a dense trace stays legible.
            const int stride = std::max(1, static_cast<int>(s.x.size()) / 14);
            p.setPen(QPen(s.style.colour, 1.5));
            p.setBrush(Qt::NoBrush);
            for (int i = 0; i < s.x.size(); i += stride) {
                draw_marker(p, s.style.marker, QPointF(px(s.x[i]), py(s.y[i])));
            }
        }

        draw_axis_titles(p, ink, box);
        draw_legend(p, ink, box, fm);

        if (!annotation_.isEmpty()) {
            p.setPen(ink);
            p.drawText(QRectF(box.left() + 6, box.top() + 4, box.width() - 12, fm.height() * 2),
                       Qt::TextWordWrap, annotation_);
        }
    }

private:
    static void draw_marker(QPainter& p, SeriesStyle::Marker m, QPointF c) {
        constexpr double r = 3.0;
        switch (m) {
            case SeriesStyle::none: break;
            case SeriesStyle::circle: p.drawEllipse(c, r, r); break;
            case SeriesStyle::square:
                p.drawRect(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r));
                break;
            case SeriesStyle::triangle: {
                QPainterPath t;
                t.moveTo(c.x(), c.y() - r);
                t.lineTo(c.x() - r, c.y() + r);
                t.lineTo(c.x() + r, c.y() + r);
                t.closeSubpath();
                p.drawPath(t);
                break;
            }
            case SeriesStyle::cross:
                p.drawLine(QPointF(c.x() - r, c.y() - r), QPointF(c.x() + r, c.y() + r));
                p.drawLine(QPointF(c.x() - r, c.y() + r), QPointF(c.x() + r, c.y() - r));
                break;
        }
    }

    static QString tick(double v) {
        if (std::abs(v) >= 1e5 || (std::abs(v) < 1e-3 && v != 0.0)) {
            return QString::number(v, 'e', 2);
        }
        return QString::number(v, 'g', 4);
    }

    void draw_axis_titles(QPainter& p, const QColor& ink, const QRect& box) const {
        p.setPen(ink);
        // §16.4: the unit is always present, in brackets, on both axes.
        p.drawText(QRect(box.left(), height() - 22, box.width(), 20), Qt::AlignCenter,
                   QString("%1 [%2]").arg(x_name_, x_unit_));
        p.save();
        p.translate(14, box.center().y());
        p.rotate(-90);
        p.drawText(QRect(-box.height() / 2, -14, box.height(), 20), Qt::AlignCenter,
                   QString("%1 [%2]").arg(y_name_, y_unit_));
        p.restore();
    }

    [[nodiscard]] int legend_width(const QFontMetrics& fm) const {
        int w = 0;
        for (const Series& s : series_) w = std::max(w, fm.horizontalAdvance(s.name));
        return series_.isEmpty() ? 0 : w + 46;
    }

    void draw_legend(QPainter& p, const QColor& ink, const QRect& box,
                     const QFontMetrics& fm) const {
        int y = box.top() + 6;
        const int x = box.right() + 10;
        for (const Series& s : series_) {
            p.setPen(QPen(s.style.colour, 2.0, s.style.line));
            p.drawLine(x, y + fm.height() / 2, x + 24, y + fm.height() / 2);
            p.setPen(QPen(s.style.colour, 1.5));
            draw_marker(p, s.style.marker, QPointF(x + 12, y + fm.height() / 2.0));
            // The series NAME is in ink, not in the series colour: §16.4 forbids a
            // label drawn in a colour close in luminance to its background, and it
            // keeps the text legible regardless of the palette.
            p.setPen(ink);
            p.drawText(x + 30, y + fm.ascent(), s.name);
            y += fm.height() + 6;
        }
    }

    [[nodiscard]] bool bounds(double& x0, double& x1, double& y0, double& y1) const {
        bool any = false;
        for (const Series& s : series_) {
            for (int i = 0; i < s.x.size(); ++i) {
                if (!std::isfinite(s.x[i]) || !std::isfinite(s.y[i])) continue;
                if (!any) {
                    x0 = x1 = s.x[i];
                    y0 = y1 = s.y[i];
                    any = true;
                } else {
                    x0 = std::min(x0, s.x[i]); x1 = std::max(x1, s.x[i]);
                    y0 = std::min(y0, s.y[i]); y1 = std::max(y1, s.y[i]);
                }
            }
        }
        if (!any) return false;
        const auto pad = [](double& lo, double& hi) {
            const double span = hi - lo;
            if (span <= 0.0) { lo -= 1.0; hi += 1.0; return; }
            lo -= 0.05 * span; hi += 0.05 * span;
        };
        pad(x0, x1);
        pad(y0, y1);
        return true;
    }

    QVector<Series> series_;
    QString x_name_ = "x", x_unit_ = "-", y_name_ = "y", y_unit_ = "-";
    QString annotation_;
    bool square_ = false, square_explicit_ = false;
};

}  // namespace rcvd::gui

#endif  // RCVD_GUI_PLOT_WIDGET_HPP
