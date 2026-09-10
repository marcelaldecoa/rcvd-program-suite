// rcvd/io/csv.hpp — §15.1 tabular output.
//
// CSV per RFC 4180, one row per output record. The header carries the channel
// name and the unit in the ACTIVE DISPLAY SYSTEM, e.g. "A_Y [g]", "Fz_LF [lbf]".
// Floating point is written with %.17g so that a round trip is lossless.
#ifndef RCVD_IO_CSV_HPP
#define RCVD_IO_CSV_HPP

#include <cstdio>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/units/convert.hpp"

namespace rcvd::io {

// One output channel: the SI values, plus the unit it is displayed in. The
// conversion is Rule U2's second and last crossing.
struct Channel {
    std::string name;
    std::string display_unit;   // must be in the closed vocabulary for `dim`
    units::Dim dim = units::Dim::Dimensionless;
    std::vector<double> si_values;

    [[nodiscard]] std::string header() const { return name + " [" + display_unit + "]"; }
};

[[nodiscard]] inline std::string csv_escape(const std::string& field) {
    const bool needs = field.find_first_of(",\"\r\n") != std::string::npos;
    if (!needs) return field;
    std::string out = "\"";
    for (char c : field) {
        if (c == '"') out += '"';   // RFC 4180: a quote is doubled
        out += c;
    }
    out += '"';
    return out;
}

// §15.1: %.17g, so a written-then-read double is bit-identical.
[[nodiscard]] inline std::string format_lossless(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

[[nodiscard]] inline core::Result<std::string> write_csv(const std::vector<Channel>& channels) {
    if (channels.empty()) {
        return core::err<std::string>("E-IO-004", "no channels to write");
    }
    const std::size_t rows = channels.front().si_values.size();
    for (const Channel& c : channels) {
        if (c.si_values.size() != rows) {
            return core::err<std::string>(
                "E-IO-004", "channel \"" + c.name + "\" has " + std::to_string(c.si_values.size()) +
                                " rows against " + std::to_string(rows) + " in the first channel");
        }
        if (!units::from_si(1.0, c.display_unit, c.dim)) {
            return core::err<std::string>(
                "E-IO-004", "channel \"" + c.name + "\" asks for display unit \"" +
                                c.display_unit + "\", which is not in the closed vocabulary for "
                                "its dimension");
        }
    }

    std::string out;
    for (std::size_t i = 0; i < channels.size(); ++i) {
        out += csv_escape(channels[i].header());
        out += (i + 1 < channels.size()) ? "," : "\r\n";   // RFC 4180 line ending
    }
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t i = 0; i < channels.size(); ++i) {
            const double display =
                *units::from_si(channels[i].si_values[r], channels[i].display_unit,
                                channels[i].dim);
            out += format_lossless(display);
            out += (i + 1 < channels.size()) ? "," : "\r\n";
        }
    }
    return out;
}

}  // namespace rcvd::io

#endif  // RCVD_IO_CSV_HPP
