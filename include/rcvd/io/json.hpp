// rcvd/io/json.hpp — a minimal JSON reader/writer for the §5 document layer.
//
// Self-contained deliberately: §4.2 lists Eigen as the only third-party runtime
// dependency, and a document format is not worth adding another. This covers
// RFC 8259 plus the // and /* */ comments the spec's own examples use (jsonc).
#ifndef RCVD_IO_JSON_HPP
#define RCVD_IO_JSON_HPP

#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "rcvd/core/result.hpp"

namespace rcvd::io {

class Json;
using JsonObject = std::map<std::string, Json>;  // ordered: Rule A2 forbids relying
                                                 // on unordered iteration anywhere
                                                 // that can affect output order
using JsonArray = std::vector<Json>;

class Json {
public:
    using Value = std::variant<std::monostate, bool, double, std::string, JsonArray, JsonObject>;

    Json() = default;
    Json(bool b) : v_(b) {}                       // NOLINT
    Json(double d) : v_(d) {}                     // NOLINT
    Json(int i) : v_(static_cast<double>(i)) {}   // NOLINT
    Json(const char* s) : v_(std::string(s)) {}   // NOLINT
    Json(std::string s) : v_(std::move(s)) {}     // NOLINT
    Json(JsonArray a) : v_(std::move(a)) {}       // NOLINT
    Json(JsonObject o) : v_(std::move(o)) {}      // NOLINT

    [[nodiscard]] bool is_null() const { return std::holds_alternative<std::monostate>(v_); }
    [[nodiscard]] bool is_bool() const { return std::holds_alternative<bool>(v_); }
    [[nodiscard]] bool is_number() const { return std::holds_alternative<double>(v_); }
    [[nodiscard]] bool is_string() const { return std::holds_alternative<std::string>(v_); }
    [[nodiscard]] bool is_array() const { return std::holds_alternative<JsonArray>(v_); }
    [[nodiscard]] bool is_object() const { return std::holds_alternative<JsonObject>(v_); }

    [[nodiscard]] bool as_bool() const { return std::get<bool>(v_); }
    [[nodiscard]] double as_number() const { return std::get<double>(v_); }
    [[nodiscard]] const std::string& as_string() const { return std::get<std::string>(v_); }
    [[nodiscard]] const JsonArray& as_array() const { return std::get<JsonArray>(v_); }
    [[nodiscard]] const JsonObject& as_object() const { return std::get<JsonObject>(v_); }
    [[nodiscard]] JsonObject& as_object() { return std::get<JsonObject>(v_); }

    // Member lookup; nullptr when absent or when this is not an object.
    [[nodiscard]] const Json* find(std::string_view key) const {
        if (!is_object()) return nullptr;
        const auto& o = as_object();
        const auto it = o.find(std::string(key));
        return it == o.end() ? nullptr : &it->second;
    }
    [[nodiscard]] bool has(std::string_view key) const { return find(key) != nullptr; }

    [[nodiscard]] std::string dump(int indent = 2) const {
        std::string s;
        write(s, indent, 0);
        return s;
    }

private:
    static void write_string(std::string& out, const std::string& s) {
        out += '"';
        for (char c : s) {
            switch (c) {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        char buf[8];
                        std::snprintf(buf, sizeof buf, "\\u%04x", c);
                        out += buf;
                    } else {
                        out += c;
                    }
            }
        }
        out += '"';
    }

    void write(std::string& out, int indent, int depth) const {
        const std::string pad(static_cast<std::size_t>(indent * (depth + 1)), ' ');
        const std::string pad_close(static_cast<std::size_t>(indent * depth), ' ');
        const char* nl = indent > 0 ? "\n" : "";

        if (is_null()) { out += "null"; return; }
        if (is_bool()) { out += as_bool() ? "true" : "false"; return; }
        if (is_number()) {
            // §15.1: %.17g so that a round trip is lossless.
            char buf[40];
            std::snprintf(buf, sizeof buf, "%.17g", as_number());
            out += buf;
            return;
        }
        if (is_string()) { write_string(out, as_string()); return; }
        if (is_array()) {
            const auto& a = as_array();
            if (a.empty()) { out += "[]"; return; }
            out += '[';
            out += nl;
            for (std::size_t i = 0; i < a.size(); ++i) {
                out += pad;
                a[i].write(out, indent, depth + 1);
                if (i + 1 < a.size()) out += ',';
                out += nl;
            }
            out += pad_close;
            out += ']';
            return;
        }
        const auto& o = as_object();
        if (o.empty()) { out += "{}"; return; }
        out += '{';
        out += nl;
        std::size_t i = 0;
        for (const auto& [k, val] : o) {
            out += pad;
            write_string(out, k);
            out += indent > 0 ? ": " : ":";
            val.write(out, indent, depth + 1);
            if (++i < o.size()) out += ',';
            out += nl;
        }
        out += pad_close;
        out += '}';
    }

    Value v_;
};

namespace detail {

struct Parser {
    std::string_view s;
    std::size_t i = 0;

    void skip() {
        for (;;) {
            while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
            // jsonc comments, which the spec's own §5 examples use.
            if (i + 1 < s.size() && s[i] == '/' && s[i + 1] == '/') {
                while (i < s.size() && s[i] != '\n') ++i;
                continue;
            }
            if (i + 1 < s.size() && s[i] == '/' && s[i + 1] == '*') {
                i += 2;
                while (i + 1 < s.size() && !(s[i] == '*' && s[i + 1] == '/')) ++i;
                i = std::min(i + 2, s.size());
                continue;
            }
            return;
        }
    }

    [[nodiscard]] std::string where() const {
        std::size_t line = 1, col = 1;
        for (std::size_t k = 0; k < i && k < s.size(); ++k) {
            if (s[k] == '\n') { ++line; col = 1; } else { ++col; }
        }
        return "line " + std::to_string(line) + ", column " + std::to_string(col);
    }
};

}  // namespace detail

[[nodiscard]] core::Result<Json> parse_json(std::string_view text);

namespace detail {

inline core::Result<Json> parse_value(Parser& p);

inline core::Result<std::string> parse_string(Parser& p) {
    if (p.i >= p.s.size() || p.s[p.i] != '"') {
        return core::err<std::string>("E-IO-001", "expected a string at " + p.where());
    }
    ++p.i;
    std::string out;
    while (p.i < p.s.size() && p.s[p.i] != '"') {
        char c = p.s[p.i];
        if (c == '\\') {
            if (++p.i >= p.s.size()) break;
            switch (p.s[p.i]) {
                case '"':  out += '"';  break;
                case '\\': out += '\\'; break;
                case '/':  out += '/';  break;
                case 'b':  out += '\b'; break;
                case 'f':  out += '\f'; break;
                case 'n':  out += '\n'; break;
                case 'r':  out += '\r'; break;
                case 't':  out += '\t'; break;
                case 'u': {
                    if (p.i + 4 >= p.s.size()) {
                        return core::err<std::string>("E-IO-001",
                                                      "truncated \\u escape at " + p.where());
                    }
                    unsigned code = 0;
                    for (int k = 1; k <= 4; ++k) {
                        const char h = p.s[p.i + static_cast<std::size_t>(k)];
                        code = code * 16 + static_cast<unsigned>(
                                   std::isdigit(static_cast<unsigned char>(h))
                                       ? h - '0'
                                       : (std::tolower(static_cast<unsigned char>(h)) - 'a' + 10));
                    }
                    p.i += 4;
                    // Enough for the ASCII/Latin-1 range these documents use.
                    if (code < 0x80) {
                        out += static_cast<char>(code);
                    } else if (code < 0x800) {
                        out += static_cast<char>(0xC0 | (code >> 6));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (code >> 12));
                        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    }
                    break;
                }
                default:
                    return core::err<std::string>(
                        "E-IO-001", "unknown escape \\" + std::string(1, p.s[p.i]) + " at " +
                                        p.where());
            }
            ++p.i;
        } else {
            out += c;
            ++p.i;
        }
    }
    if (p.i >= p.s.size()) {
        return core::err<std::string>("E-IO-001", "unterminated string at " + p.where());
    }
    ++p.i;  // closing quote
    return out;
}

inline core::Result<Json> parse_value(Parser& p) {
    p.skip();
    if (p.i >= p.s.size()) {
        return core::err<Json>("E-IO-001", "unexpected end of document at " + p.where());
    }

    const char c = p.s[p.i];
    if (c == '{') {
        ++p.i;
        JsonObject obj;
        p.skip();
        if (p.i < p.s.size() && p.s[p.i] == '}') { ++p.i; return Json(std::move(obj)); }
        for (;;) {
            p.skip();
            auto key = parse_string(p);
            if (!key.ok()) return core::Result<Json>(key.error());
            p.skip();
            if (p.i >= p.s.size() || p.s[p.i] != ':') {
                return core::err<Json>("E-IO-001", "expected ':' at " + p.where());
            }
            ++p.i;
            auto val = parse_value(p);
            if (!val.ok()) return val;
            // A duplicate key is a document defect, not a last-one-wins.
            if (obj.count(key.value())) {
                return core::err<Json>("E-IO-001",
                                       "duplicate key '" + key.value() + "' at " + p.where());
            }
            obj.emplace(key.value(), std::move(val).value());
            p.skip();
            if (p.i < p.s.size() && p.s[p.i] == ',') { ++p.i; p.skip();
                if (p.i < p.s.size() && p.s[p.i] == '}') { ++p.i; return Json(std::move(obj)); }
                continue; }
            if (p.i < p.s.size() && p.s[p.i] == '}') { ++p.i; return Json(std::move(obj)); }
            return core::err<Json>("E-IO-001", "expected ',' or '}' at " + p.where());
        }
    }
    if (c == '[') {
        ++p.i;
        JsonArray arr;
        p.skip();
        if (p.i < p.s.size() && p.s[p.i] == ']') { ++p.i; return Json(std::move(arr)); }
        for (;;) {
            auto val = parse_value(p);
            if (!val.ok()) return val;
            arr.push_back(std::move(val).value());
            p.skip();
            if (p.i < p.s.size() && p.s[p.i] == ',') { ++p.i; continue; }
            if (p.i < p.s.size() && p.s[p.i] == ']') { ++p.i; return Json(std::move(arr)); }
            return core::err<Json>("E-IO-001", "expected ',' or ']' at " + p.where());
        }
    }
    if (c == '"') {
        auto sv = parse_string(p);
        if (!sv.ok()) return core::Result<Json>(sv.error());
        return Json(std::move(sv).value());
    }
    if (p.s.compare(p.i, 4, "true") == 0)  { p.i += 4; return Json(true); }
    if (p.s.compare(p.i, 5, "false") == 0) { p.i += 5; return Json(false); }
    if (p.s.compare(p.i, 4, "null") == 0)  { p.i += 4; return Json(); }

    // Number. §13.1 makes a NaN a fatal defect, so NaN and Infinity literals are
    // rejected rather than parsed: a document must not be able to inject one.
    const std::size_t start = p.i;
    if (p.i < p.s.size() && (p.s[p.i] == '-' || p.s[p.i] == '+')) ++p.i;
    while (p.i < p.s.size() && (std::isdigit(static_cast<unsigned char>(p.s[p.i])) ||
                                p.s[p.i] == '.' || p.s[p.i] == 'e' || p.s[p.i] == 'E' ||
                                p.s[p.i] == '+' || p.s[p.i] == '-')) {
        if ((p.s[p.i] == '+' || p.s[p.i] == '-') && p.i > start &&
            !(p.s[p.i - 1] == 'e' || p.s[p.i - 1] == 'E')) {
            break;
        }
        ++p.i;
    }
    if (p.i == start) {
        return core::err<Json>("E-IO-001",
                               "unexpected character '" + std::string(1, c) + "' at " + p.where());
    }
    const std::string text(p.s.substr(start, p.i - start));
    try {
        std::size_t consumed = 0;
        const double d = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(d)) {
            return core::err<Json>("E-IO-001", "not a finite number: '" + text + "'");
        }
        return Json(d);
    } catch (...) {
        return core::err<Json>("E-IO-001", "malformed number '" + text + "' at " + p.where());
    }
}

}  // namespace detail

[[nodiscard]] inline core::Result<Json> parse_json(std::string_view text) {
    detail::Parser p{text, 0};
    auto v = detail::parse_value(p);
    if (!v.ok()) return v;
    p.skip();
    if (p.i != p.s.size()) {
        return core::err<Json>("E-IO-001", "trailing content after the document at " + p.where());
    }
    return v;
}

}  // namespace rcvd::io

#endif  // RCVD_IO_JSON_HPP
