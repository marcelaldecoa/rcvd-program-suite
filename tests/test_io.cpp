// Spec §5 (document layer), §15.1 (tabular output) and §4.5 (run manifest).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdlib>
#include <limits>

#include "rcvd/io/csv.hpp"
#include "rcvd/io/document.hpp"
#include "rcvd/io/manifest.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::io;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ---------------------------------------------------------------------- JSON
TEST_CASE("JSON round-trips losslessly", "[io][json]") {
    const char* text = R"JSON({
        "schema_version": "1.0",
        "name": "CH5",
        "nested": {"a": [1, 2.5, -3e4], "b": true, "c": null},
        "escaped": "quote \" backslash \\ newline \n"
    })JSON";
    const auto doc = parse_json(text);
    REQUIRE(doc.ok());
    REQUIRE(doc.value().find("name")->as_string() == "CH5");
    REQUIRE(doc.value().find("nested")->find("b")->as_bool());
    REQUIRE(doc.value().find("nested")->find("c")->is_null());
    REQUIRE_THAT(doc.value().find("nested")->find("a")->as_array()[2].as_number(),
                 WithinRel(-3e4, 1e-15));

    // §15.1's %.17g applies to JSON too: parse(dump(x)) must return x exactly.
    const auto again = parse_json(doc.value().dump());
    REQUIRE(again.ok());
    REQUIRE(again.value().dump() == doc.value().dump());
    REQUIRE(again.value().find("escaped")->as_string() ==
            doc.value().find("escaped")->as_string());
}

TEST_CASE("JSON preserves doubles bit-exactly", "[io][json]") {
    for (double v : {0.1, 1.0 / 3.0, 1e-300, 1.7976931348623157e308,
                     -2.2250738585072014e-308, 9007199254740993.0}) {
        JsonObject o;
        o.emplace("x", Json(v));
        const auto back = parse_json(Json(std::move(o)).dump());
        REQUIRE(back.ok());
        INFO("value " << v);
        REQUIRE(back.value().find("x")->as_number() == v);   // bit-identical
    }
}

TEST_CASE("JSON rejects malformed documents", "[io][json]") {
    for (const char* bad : {"{", "{\"a\":}", "{\"a\" 1}", "[1,2", "{\"a\":1} trailing",
                            "{\"a\":1,\"a\":2}"}) {
        INFO("input " << bad);
        REQUIRE_FALSE(parse_json(bad).ok());
    }
    // §13.1 makes a NaN a fatal defect, so a document must not be able to inject
    // one through a literal.
    REQUIRE_FALSE(parse_json("{\"x\": NaN}").ok());
    REQUIRE_FALSE(parse_json("{\"x\": Infinity}").ok());
}

TEST_CASE("JSON accepts the comments the §5 examples use", "[io][json]") {
    const char* text = R"JSON({
        // a line comment, as in the §5.1 example
        "schema_version": "1.0",   /* and a block one */
        "x": 1
    })JSON";
    const auto doc = parse_json(text);
    REQUIRE(doc.ok());
    REQUIRE_THAT(doc.value().find("x")->as_number(), WithinRel(1.0, 1e-15));
}

// ------------------------------------------------------------------ §5 schema
TEST_CASE("§5: major schema-version mismatch is rejected", "[io][document]") {
    const auto make = [](const char* ver) {
        return parse_json(std::string(R"JSON({"schema_version": ")JSON") + ver +
                          R"JSON(", "inertia": {}, "geometry": {}})JSON")
            .value();
    };
    REQUIRE_FALSE(check_schema_version(make("1.0"), "doc").has_value());
    REQUIRE_FALSE(check_schema_version(make("1.7"), "doc").has_value());   // minor is fine
    REQUIRE(check_schema_version(make("2.0"), "doc").has_value());
    REQUIRE(check_schema_version(make("0.9"), "doc").has_value());
    // Missing entirely.
    REQUIRE(check_schema_version(parse_json("{}").value(), "doc").has_value());
}

TEST_CASE("§5.2: a dimensional coefficient without a unit is rejected", "[io][document]") {
    // This is the rule that stops the source's English-unit expressions being read
    // as SI — a factor of 57 in c0, 13 in c1 and 4.4 in mu1.
    const auto bare = parse_json(R"JSON({"c0": -0.4})JSON").value();
    const auto r = quantity(bare, "c0", units::Dim::PerAngle, "tire");
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-IO-003");
    REQUIRE(r.error().message.find("bare number") != std::string::npos);

    // With the unit it converts, and by exactly the factor §5.2 names.
    const auto withu = parse_json(R"JSON({"c0": {"value": -0.4, "unit": "1/deg"}})JSON").value();
    const auto ok = quantity(withu, "c0", units::Dim::PerAngle, "tire");
    REQUIRE(ok.ok());
    REQUIRE_THAT(ok.value() / -0.4, WithinRel(1.0 / units::DEGREE, 1e-12));
    REQUIRE_THAT(std::abs(ok.value() / -0.4), WithinRel(57.2958, 1e-4));

    // A unit from the wrong dimension is rejected rather than silently applied.
    const auto wrong = parse_json(R"JSON({"c0": {"value": -0.4, "unit": "lbf"}})JSON").value();
    REQUIRE_FALSE(quantity(wrong, "c0", units::Dim::PerAngle, "tire").ok());
}

TEST_CASE("§5.1: a vehicle document loads and converts", "[io][document]") {
    const char* text = R"JSON({
      "schema_version": "1.0",
      "name": "CH5",
      "inertia": {
        "weight": {"value": 3700, "unit": "lbf"},
        "yaw_inertia": {"value": 1800, "unit": "slug*ft^2"},
        "unsprung_weight_front": {"value": 240, "unit": "lbf"},
        "unsprung_weight_rear":  {"value": 260, "unit": "lbf"},
        "unsprung_cg_height_front": {"value": 11, "unit": "in"},
        "unsprung_cg_height_rear":  {"value": 11, "unit": "in"}
      },
      "geometry": {
        "wheelbase": {"value": 104.5, "unit": "in"},
        "front_weight_fraction": 0.57,
        "track_front": {"value": 60, "unit": "in"},
        "track_rear":  {"value": 60, "unit": "in"},
        "cg_height":   {"value": 20, "unit": "in"}
      },
      "steering": {"ratio": 14.0}
    })JSON";
    const auto doc = parse_json(text);
    REQUIRE(doc.ok());
    const auto in = load_vehicle(doc.value());
    REQUIRE(in.ok());

    const auto v = core::resolve(in.value());
    REQUIRE(v.ok());
    // The same numbers B-01/B-02 assert, now arriving through the document layer.
    REQUIRE_THAT(v.value().m / units::SLUG, WithinRel(115.0, 1e-5));
    REQUIRE_THAT(v.value().a / units::FOOT, WithinRel(3.7446, 1e-4));
    REQUIRE_THAT(v.value().b / units::FOOT, WithinRel(4.9637, 1e-4));
    // And (5.1b)/(5.1c) derive the §5.1 worked example's h_s and a_s.
    REQUIRE_THAT(v.value().h_s / units::INCH, WithinRel(21.406, 1e-4));
    REQUIRE_THAT(v.value().a_s / units::INCH, WithinRel(43.466, 1e-4));
}

TEST_CASE("§5.1: over-specification is rejected at the document layer",
          "[io][document][validation]") {
    // Both mass and weight.
    const auto both = parse_json(R"JSON({
      "schema_version": "1.0",
      "inertia": {"mass": {"value": 1500, "unit": "kg"},
                  "weight": {"value": 3700, "unit": "lbf"}},
      "geometry": {"wheelbase": {"value": 104.5, "unit": "in"},
                   "front_weight_fraction": 0.57}
    })JSON");
    REQUIRE(both.ok());
    const auto in = load_vehicle(both.value());
    REQUIRE(in.ok());                       // the loader fills both...
    REQUIRE_FALSE(core::resolve(in.value()).ok());   // ...and resolve rejects it
    REQUIRE(core::resolve(in.value()).error().code == "E-VEH-010");

    // Both springing groups: caught in the loader, since only it can see the
    // component keys.
    const auto spr = parse_json(R"JSON({
      "schema_version": "1.0",
      "inertia": {"weight": {"value": 3700, "unit": "lbf"}},
      "geometry": {"wheelbase": {"value": 104.5, "unit": "in"},
                   "front_weight_fraction": 0.57},
      "springing": {
        "roll_stiffness_chassis_to_ground_front": {"value": 975, "unit": "lbf*ft/deg"},
        "roll_stiffness_chassis_to_ground_rear":  {"value": 325, "unit": "lbf*ft/deg"},
        "wheel_centre_rate_front": {"value": 183.627, "unit": "lbf/in"}
      }
    })JSON");
    REQUIRE(spr.ok());
    const auto r = load_vehicle(spr.value());
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-VEH-010");
}

TEST_CASE("§5.2: a tire document loads, and the redundancy rules hold",
          "[io][document][validation]") {
    // The §5.2 example as CORRECTED in this repo's spec commit: one source of
    // cornering stiffness, and no friction block on a pure-stiffness model.
    const char* good = R"JSON({
      "schema_version": "1.0",
      "id": "sedan_front",
      "model": "linear_breakaway",
      "linear": {"cornering_stiffness": {"value": -640, "unit": "lbf/deg"},
                 "breakaway_slip": {"value": 5, "unit": "deg"}},
      "pneumatic_trail": {"value": 1.1, "unit": "in"}
    })JSON";
    const auto cfg = load_linear_tire(parse_json(good).value());
    REQUIRE(cfg.ok());
    const auto t = tire::LinearTire::create(cfg.value());
    REQUIRE(t.ok());
    REQUIRE_THAT(t.value().evaluate({0.01, 5000.0}).Fy_N,
                 WithinRel(-640.0 * units::LBF_PER_DEG_TO_N_PER_RAD * 0.01, 1e-12));

    // E-TIRE-005: two independent sources of cornering stiffness.
    const char* two_sources = R"JSON({
      "schema_version": "1.0", "id": "x", "model": "linear_breakaway",
      "linear": {"cornering_stiffness": {"value": -640, "unit": "lbf/deg"},
                 "breakaway_slip": {"value": 5, "unit": "deg"}},
      "load_sensitivity": {"mode": "quadratic",
                           "c0": {"value": -0.4, "unit": "1/deg"},
                           "c1": {"value": 8e-5, "unit": "1/(lbf*deg)"}}
    })JSON";
    const auto two = load_linear_tire(parse_json(two_sources).value());
    REQUIRE(two.ok());
    REQUIRE(tire::LinearTire::create(two.value()).error().code == "E-TIRE-005");

    // E-TIRE-006: a friction block on a model with has_mu() == false.
    const char* with_friction = R"JSON({
      "schema_version": "1.0", "id": "x", "model": "linear_breakaway",
      "linear": {"cornering_stiffness": {"value": -640, "unit": "lbf/deg"},
                 "breakaway_slip": {"value": 5, "unit": "deg"}},
      "friction": {"mode": "linear", "mu0": 1.22, "mu1": {"value": 0.00024, "unit": "1/lbf"}}
    })JSON";
    const auto fr = load_linear_tire(parse_json(with_friction).value());
    REQUIRE(fr.ok());
    REQUIRE(fr.value().document_has_friction_block);
    REQUIRE(tire::LinearTire::create(fr.value()).error().code == "E-TIRE-006");

    // model/breakaway consistency, both directions.
    const char* mismatched = R"JSON({
      "schema_version": "1.0", "id": "x", "model": "linear",
      "linear": {"cornering_stiffness": {"value": -640, "unit": "lbf/deg"},
                 "breakaway_slip": {"value": 5, "unit": "deg"}}
    })JSON";
    REQUIRE_FALSE(load_linear_tire(parse_json(mismatched).value()).ok());
}

// ------------------------------------------------------------------ §15.1 CSV
TEST_CASE("§15.1: CSV headers carry units and values round-trip losslessly",
          "[io][csv]") {
    Channel t{"time", "s", units::Dim::Time, {0.0, 0.01, 0.02}};
    Channel ay{"A_Y", "g", units::Dim::Acceleration,
               {0.0, 1.0 / 3.0, 9.80665}};
    Channel fz{"Fz_LF", "lbf", units::Dim::Force,
               {1000.0, 4448.2216152605, 0.1}};

    const auto csv = write_csv({t, ay, fz});
    REQUIRE(csv.ok());
    const std::string& s = csv.value();
    REQUIRE(s.find("time [s],A_Y [g],Fz_LF [lbf]\r\n") == 0);

    // Parse the second data row back and confirm the displayed values are exact.
    const std::size_t l1 = s.find("\r\n") + 2;
    const std::size_t l2 = s.find("\r\n", l1) + 2;
    const std::string row = s.substr(l2, s.find("\r\n", l2) - l2);
    const std::size_t c1 = row.find(','), c2 = row.find(',', c1 + 1);
    REQUIRE(std::strtod(row.substr(0, c1).c_str(), nullptr) == 0.01);
    // 1/3 m/s^2 in g, written at %.17g, must read back bit-identically.
    REQUIRE(std::strtod(row.substr(c1 + 1, c2 - c1 - 1).c_str(), nullptr) ==
            (1.0 / 3.0) / units::G_STANDARD);
    REQUIRE(std::strtod(row.substr(c2 + 1).c_str(), nullptr) ==
            4448.2216152605 / units::POUND_FORCE);
}

TEST_CASE("§15.1: CSV rejects a display unit outside the vocabulary", "[io][csv]") {
    Channel bad{"A_Y", "furlongs_per_fortnight", units::Dim::Acceleration, {1.0}};
    REQUIRE_FALSE(write_csv({bad}).ok());
    // ...and a dimension mismatch, which is the same class of error.
    Channel mismatched{"A_Y", "lbf", units::Dim::Acceleration, {1.0}};
    REQUIRE_FALSE(write_csv({mismatched}).ok());
    // Ragged channels are a defect, not a truncation.
    Channel a{"a", "s", units::Dim::Time, {1.0, 2.0}};
    Channel b{"b", "s", units::Dim::Time, {1.0}};
    REQUIRE_FALSE(write_csv({a, b}).ok());
}

TEST_CASE("RFC 4180 escaping", "[io][csv]") {
    REQUIRE(csv_escape("plain") == "plain");
    REQUIRE(csv_escape("has,comma") == "\"has,comma\"");
    REQUIRE(csv_escape("has\"quote") == "\"has\"\"quote\"");
}

// --------------------------------------------------------------- §4.5 manifest
TEST_CASE("§4.5: the manifest records the resolved SI input", "[io][manifest]") {
    const auto v = core::resolve(fixtures::ch5_input());
    REQUIRE(v.ok());

    RunManifest m;
    m.build.git_sha = "deadbeef";
    m.build.compiler = "MSVC 19.44";
    m.build.compile_flags = "/std:c++20 /fp:precise";
    m.build.platform = "windows-x86_64";
    m.resolved_input = resolved_vehicle_json(v.value());
    m.wall_time_s = 0.0123;
    m.integrator.accepted = 512;
    m.integrator.rejected = 3;
    m.output_sha256 = "abc123";
    m.diagnostics.push_back(core::make_diagnostic("EV-LIFT", "inside rear lifted", 1.25,
                                                  core::Wheel::RR));

    const Json j = manifest_json(m);
    // The recorded input is post-conversion SI, so it carries no unit strings and
    // is directly reproducible.
    REQUIRE_THAT(j.find("resolved_input")->find("geometry")->find("a_m")->as_number(),
                 WithinRel(v.value().a, 1e-15));
    REQUIRE(j.find("resolved_input")->find("units")->as_string().find("SI") == 0);
    REQUIRE(j.find("build")->find("git_sha")->as_string() == "deadbeef");
    REQUIRE_THAT(j.find("integrator")->find("steps_accepted")->as_number(),
                 WithinAbs(512.0, 0.0));

    // Rule U4: gravity is recorded, and flagged when it is not standard.
    REQUIRE(j.find("gravity_is_standard")->as_bool());
    m.gravity_mps2 = 9.79;
    REQUIRE_FALSE(manifest_json(m).find("gravity_is_standard")->as_bool());

    // Diagnostics survive with their wheel and time.
    const auto& d = j.find("diagnostics")->as_array();
    REQUIRE(d.size() == 1);
    REQUIRE(d[0].find("code")->as_string() == "EV-LIFT");
    REQUIRE(d[0].find("wheel")->as_string() == "RR");

    // And the whole manifest round-trips as JSON.
    const auto back = parse_json(j.dump());
    REQUIRE(back.ok());
    REQUIRE(back.value().dump() == j.dump());
}
