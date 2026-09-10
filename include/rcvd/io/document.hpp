// rcvd/io/document.hpp — the §5 document layer.
//
// Rule U2: unit conversion happens exactly twice per data path, once on
// deserialisation and once on display. This is the deserialisation half; nothing
// below this line ever sees a foot or a pound.
#ifndef RCVD_IO_DOCUMENT_HPP
#define RCVD_IO_DOCUMENT_HPP

#include <optional>
#include <string>

#include "rcvd/core/vehicle.hpp"
#include "rcvd/io/json.hpp"
#include "rcvd/tire/linear.hpp"
#include "rcvd/units/convert.hpp"

namespace rcvd::io {

inline constexpr int kSchemaMajor = 1;

// §5: every document carries "schema_version" and SHALL be rejected on
// major-version mismatch.
[[nodiscard]] inline std::optional<core::Diagnostic> check_schema_version(const Json& doc,
                                                                          const std::string& what) {
    const Json* v = doc.find("schema_version");
    if (!v || !v->is_string()) {
        return core::make_diagnostic("E-IO-002",
                                     what + ": missing or non-string \"schema_version\"");
    }
    const std::string& s = v->as_string();
    const std::size_t dot = s.find('.');
    const std::string major = dot == std::string::npos ? s : s.substr(0, dot);
    if (major != std::to_string(kSchemaMajor)) {
        return core::make_diagnostic(
            "E-IO-002", what + ": schema_version \"" + s + "\" has major version " + major +
                            ", but this build implements major version " +
                            std::to_string(kSchemaMajor));
    }
    return std::nullopt;
}

// §5: physical quantities are {"value": <number>, "unit": "<string>"}, with the
// unit parsed against the closed vocabulary.
//
// §5.2 is emphatic that a DIMENSIONAL coefficient given as a bare number must be
// rejected: reading the source's English-unit expressions as SI is a factor-of-57
// error in c0, 13 in c1 and 4.4 in mu1. So there is deliberately no "assume SI"
// fallback here.
[[nodiscard]] inline core::Result<double> quantity(const Json& parent, std::string_view key,
                                                   units::Dim dim, const std::string& what) {
    const Json* q = parent.find(key);
    if (!q) {
        return core::err<double>("E-IO-003",
                                 what + ": missing required quantity \"" + std::string(key) + "\"");
    }
    if (q->is_number()) {
        return core::err<double>(
            "E-IO-003",
            what + ": \"" + std::string(key) +
                "\" is a dimensional quantity given as a bare number; it must be written as "
                "{\"value\": ..., \"unit\": ...}. Reading an English-unit coefficient as SI is "
                "a silent factor error, which is why §5.2 forbids the bare form.");
    }
    if (!q->is_object()) {
        return core::err<double>("E-IO-003",
                                 what + ": \"" + std::string(key) + "\" must be an object");
    }
    const Json* val = q->find("value");
    const Json* unit = q->find("unit");
    if (!val || !val->is_number()) {
        return core::err<double>("E-IO-003",
                                 what + ": \"" + std::string(key) + "\" has no numeric \"value\"");
    }
    if (!unit || !unit->is_string()) {
        return core::err<double>("E-IO-003",
                                 what + ": \"" + std::string(key) + "\" has no \"unit\" string");
    }
    const auto si = units::to_si(val->as_number(), unit->as_string(), dim);
    if (!si) {
        return core::err<double>(
            "E-IO-003", what + ": \"" + std::string(key) + "\" has unit \"" + unit->as_string() +
                            "\", which is not in the closed vocabulary for its dimension");
    }
    return *si;
}

[[nodiscard]] inline core::Result<double> optional_quantity(const Json& parent,
                                                            std::string_view key, units::Dim dim,
                                                            const std::string& what,
                                                            bool& present) {
    present = parent.has(key);
    if (!present) return 0.0;
    return quantity(parent, key, dim, what);
}

// ------------------------------------------------------------------ §5.1 Vehicle
[[nodiscard]] inline core::Result<core::VehicleInput> load_vehicle(const Json& doc) {
    const std::string what = "vehicle document";
    if (auto d = check_schema_version(doc, what)) return core::Result<core::VehicleInput>(*d);

    core::VehicleInput in;
    if (const Json* n = doc.find("name"); n && n->is_string()) in.name = n->as_string();

    const Json* inertia = doc.find("inertia");
    const Json* geometry = doc.find("geometry");
    if (!inertia || !geometry) {
        return core::err<core::VehicleInput>(
            "E-IO-003", what + ": both \"inertia\" and \"geometry\" are required");
    }

    // Exactly one of mass / weight (§5.1 redundancy rule).
    bool has_mass = false, has_weight = false;
    auto mass = optional_quantity(*inertia, "mass", units::Dim::Mass, what, has_mass);
    if (!mass.ok()) return core::Result<core::VehicleInput>(mass.error());
    auto weight = optional_quantity(*inertia, "weight", units::Dim::Force, what, has_weight);
    if (!weight.ok()) return core::Result<core::VehicleInput>(weight.error());
    if (has_mass) in.mass_kg = mass.value();
    if (has_weight) in.weight_N = weight.value();

    bool present = false;
    auto yaw = optional_quantity(*inertia, "yaw_inertia", units::Dim::Inertia, what, present);
    if (!yaw.ok()) return core::Result<core::VehicleInput>(yaw.error());
    in.yaw_inertia_kg_m2 = yaw.value();

    // Unsprung masses are PER AXLE (§5.1); the field names say so.
    struct Field { const char* key; units::Dim dim; double* dst; };
    auto load_optional = [&](const Json& parent, const Field& f) -> std::optional<core::Diagnostic> {
        bool p = false;
        auto q = optional_quantity(parent, f.key, f.dim, what, p);
        if (!q.ok()) return q.error();
        if (p) *f.dst = q.value();
        return std::nullopt;
    };

    // Accept unsprung mass either as a weight or a mass, converting on the spot.
    for (const auto& [key, dst] : {std::pair<const char*, double*>{"unsprung_weight_front",
                                                                   &in.unsprung_mass_front_kg},
                                   {"unsprung_weight_rear", &in.unsprung_mass_rear_kg}}) {
        bool p = false;
        auto q = optional_quantity(*inertia, key, units::Dim::Force, what, p);
        if (!q.ok()) return core::Result<core::VehicleInput>(q.error());
        if (p) *dst = q.value() / units::G_STANDARD;
    }
    for (const Field& f : {Field{"unsprung_cg_height_front", units::Dim::Length,
                                 &in.unsprung_cg_height_front_m},
                           Field{"unsprung_cg_height_rear", units::Dim::Length,
                                 &in.unsprung_cg_height_rear_m}}) {
        if (auto d = load_optional(*inertia, f)) return core::Result<core::VehicleInput>(*d);
    }

    // Geometry: exactly one of {wheelbase, front_weight_fraction} / {a, b}.
    bool has_wb = false, has_a = false, has_b = false;
    auto wb = optional_quantity(*geometry, "wheelbase", units::Dim::Length, what, has_wb);
    if (!wb.ok()) return core::Result<core::VehicleInput>(wb.error());
    if (has_wb) in.wheelbase_m = wb.value();
    if (const Json* f = geometry->find("front_weight_fraction"); f && f->is_number()) {
        in.front_weight_fraction = f->as_number();
    }
    auto a = optional_quantity(*geometry, "a", units::Dim::Length, what, has_a);
    if (!a.ok()) return core::Result<core::VehicleInput>(a.error());
    if (has_a) in.a_m = a.value();
    auto b = optional_quantity(*geometry, "b", units::Dim::Length, what, has_b);
    if (!b.ok()) return core::Result<core::VehicleInput>(b.error());
    if (has_b) in.b_m = b.value();

    for (const Field& f : {Field{"track_front", units::Dim::Length, &in.track_front_m},
                           Field{"track_rear", units::Dim::Length, &in.track_rear_m},
                           Field{"spring_track_front", units::Dim::Length, &in.spring_track_front_m},
                           Field{"spring_track_rear", units::Dim::Length, &in.spring_track_rear_m},
                           Field{"cg_height", units::Dim::Length, &in.cg_height_m},
                           Field{"roll_centre_height_front", units::Dim::Length,
                                 &in.roll_centre_height_front_m},
                           Field{"roll_centre_height_rear", units::Dim::Length,
                                 &in.roll_centre_height_rear_m}}) {
        if (auto d = load_optional(*geometry, f)) return core::Result<core::VehicleInput>(*d);
    }

    // h_s and a_s are optional; (5.1b)/(5.1c) check them when supplied.
    bool p = false;
    auto hs = optional_quantity(*geometry, "sprung_cg_height", units::Dim::Length, what, p);
    if (!hs.ok()) return core::Result<core::VehicleInput>(hs.error());
    if (p) in.sprung_cg_height_m = hs.value();
    auto as = optional_quantity(*geometry, "sprung_cg_front_distance", units::Dim::Length, what, p);
    if (!as.ok()) return core::Result<core::VehicleInput>(as.error());
    if (p) in.sprung_cg_front_distance_m = as.value();

    // Springing: chassis-to-ground roll stiffnesses (decision D-5).
    if (const Json* spr = doc.find("springing")) {
        bool pf = false, pr = false;
        auto kf = optional_quantity(*spr, "roll_stiffness_chassis_to_ground_front",
                                    units::Dim::AngularStiffness, what, pf);
        if (!kf.ok()) return core::Result<core::VehicleInput>(kf.error());
        auto kr = optional_quantity(*spr, "roll_stiffness_chassis_to_ground_rear",
                                    units::Dim::AngularStiffness, what, pr);
        if (!kr.ok()) return core::Result<core::VehicleInput>(kr.error());
        if (pf) in.roll_stiffness_ctg_front_Nm_per_rad = kf.value();
        if (pr) in.roll_stiffness_ctg_rear_Nm_per_rad = kr.value();

        // §5.1: the chassis-to-ground group and the P6 component group are
        // alternatives, never both (E-VEH-010).
        static constexpr const char* kComponentKeys[] = {
            "wheel_centre_rate_front", "wheel_centre_rate_rear", "aux_roll_stiffness_front",
            "aux_roll_stiffness_rear", "tire_rate_front", "tire_rate_rear"};
        bool any_component = false;
        for (const char* k : kComponentKeys) any_component |= spr->has(k);
        if ((pf || pr) && any_component) {
            return core::err<core::VehicleInput>(
                "E-VEH-010",
                what + ": \"springing\" supplies both the chassis-to-ground roll stiffnesses "
                       "and the P6 component group; these are alternatives, not a base plus a "
                       "modifier");
        }
    }

    if (const Json* st = doc.find("steering")) {
        if (const Json* ratio = st->find("ratio"); ratio && ratio->is_number()) {
            in.steering_ratio = ratio->as_number();
        }
    }
    return in;
}

// -------------------------------------------------------------------- §5.2 Tire
[[nodiscard]] inline core::Result<tire::LinearTireConfig> load_linear_tire(const Json& doc) {
    const std::string what = "tire document";
    if (auto d = check_schema_version(doc, what)) {
        return core::Result<tire::LinearTireConfig>(*d);
    }

    tire::LinearTireConfig cfg;
    if (const Json* id = doc.find("id"); id && id->is_string()) cfg.id = id->as_string();

    const Json* model = doc.find("model");
    if (!model || !model->is_string()) {
        return core::err<tire::LinearTireConfig>("E-IO-003", what + ": missing \"model\"");
    }
    const std::string& kind = model->as_string();
    if (kind != "linear" && kind != "linear_breakaway") {
        return core::err<tire::LinearTireConfig>(
            "E-IO-003", what + ": model \"" + kind + "\" is not one this loader handles "
                               "(linear | linear_breakaway)");
    }

    bool present = false;
    if (const Json* lin = doc.find("linear")) {
        auto ca = optional_quantity(*lin, "cornering_stiffness", units::Dim::CorneringStiffness,
                                    what, present);
        if (!ca.ok()) return core::Result<tire::LinearTireConfig>(ca.error());
        if (present) cfg.C_alpha_N_per_rad = ca.value();

        auto bk = optional_quantity(*lin, "breakaway_slip", units::Dim::Angle, what, present);
        if (!bk.ok()) return core::Result<tire::LinearTireConfig>(bk.error());
        if (present) cfg.breakaway_slip_rad = bk.value();
    }
    if (kind == "linear_breakaway" && !cfg.breakaway_slip_rad) {
        return core::err<tire::LinearTireConfig>(
            "E-IO-003", what + ": model \"linear_breakaway\" requires linear.breakaway_slip");
    }
    if (kind == "linear" && cfg.breakaway_slip_rad) {
        return core::err<tire::LinearTireConfig>(
            "E-IO-003", what + ": model \"linear\" must not supply a breakaway_slip; use "
                               "\"linear_breakaway\"");
    }

    auto tp = optional_quantity(doc, "pneumatic_trail", units::Dim::Length, what, present);
    if (!tp.ok()) return core::Result<tire::LinearTireConfig>(tp.error());
    if (present) cfg.pneumatic_trail_m = tp.value();

    // (6.10). Both coefficients are dimensional and both must carry units.
    if (const Json* ls = doc.find("load_sensitivity")) {
        const Json* mode = ls->find("mode");
        if (!mode || !mode->is_string() || mode->as_string() != "quadratic") {
            return core::err<tire::LinearTireConfig>(
                "E-IO-003", what + ": load_sensitivity.mode must be \"quadratic\" for this model");
        }
        auto c0 = quantity(*ls, "c0", units::Dim::PerAngle, what);
        if (!c0.ok()) return core::Result<tire::LinearTireConfig>(c0.error());
        auto c1 = quantity(*ls, "c1", units::Dim::PerForceAngle, what);
        if (!c1.ok()) return core::Result<tire::LinearTireConfig>(c1.error());
        cfg.load_sensitivity = tire::QuadraticStiffness{c0.value(), c1.value()};
    }

    // §6.1 T5 / E-TIRE-006: a friction block is inadmissible for a pure-stiffness
    // model. Recorded here and rejected at construction, so the code lives in one
    // place.
    cfg.document_has_friction_block = doc.has("friction");

    auto fz = optional_quantity(doc, "max_operating_load", units::Dim::Force, what, present);
    if (!fz.ok()) return core::Result<tire::LinearTireConfig>(fz.error());
    if (present) cfg.Fz_max_operating_N = fz.value();

    return cfg;
}

}  // namespace rcvd::io

#endif  // RCVD_IO_DOCUMENT_HPP
