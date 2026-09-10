// rcvd/models/plant.hpp — the Plant contract (§4.3), driver inputs and the
// maneuver schedule (§5.6), and SolverConfig's model-reduction switches (§5.5).
#ifndef RCVD_MODELS_PLANT_HPP
#define RCVD_MODELS_PLANT_HPP

#include <cmath>
#include <span>
#include <vector>

#include "rcvd/numerics/pchip.hpp"
#include "rcvd/units/constants.hpp"

namespace rcvd::models {

// The EVALUATED input at time t (§5.7), as distinct from the Maneuver schedule.
struct DriverInput {
    double delta_rad = 0.0;
    double speed_cmd_mps = 0.0;
};

enum class SpeedMode {
    constant,        // u is held; the longitudinal equation is not integrated
    constant_accel,  // §9.6 "kinematic speed schedule", not a dynamics
};

// §5.6.
class Maneuver {
public:
    enum class Kind { fixed, step, ramp, sinusoid, doublet, table };

    static Maneuver make_fixed(double A) { return Maneuver(Kind::fixed, A); }
    static Maneuver make_step(double A, double t0) {
        Maneuver m(Kind::step, A);
        m.t0_ = t0;
        return m;
    }
    static Maneuver make_ramp(double A, double t0, double rise_s) {
        Maneuver m(Kind::ramp, A);
        m.t0_ = t0;
        m.T_ = rise_s;
        return m;
    }
    static Maneuver make_sinusoid(double A, double period_s) {
        Maneuver m(Kind::sinusoid, A);
        m.T_ = period_s;
        return m;
    }
    static Maneuver make_doublet(double A, double t0, double period_s) {
        Maneuver m(Kind::doublet, A);
        m.t0_ = t0;
        m.T_ = period_s;
        return m;
    }
    // §13.5: monotone cubic Hermite, so a tabulated input is C1 and does not
    // corrupt the integrator's error estimate at every node.
    static Maneuver make_table(std::span<const double> t, std::span<const double> delta) {
        Maneuver m(Kind::table, 0.0);
        m.table_ = numerics::Pchip(t, delta);
        return m;
    }

    [[nodiscard]] double delta_at(double t) const {
        switch (kind_) {
            case Kind::fixed:
                return A_;
            case Kind::step:
                return (t >= t0_) ? A_ : 0.0;
            case Kind::ramp:
                return A_ * std::clamp((t - t0_) / T_, 0.0, 1.0);
            case Kind::sinusoid:
                return A_ * std::sin(2.0 * units::PI * t / T_);
            case Kind::doublet: {
                const auto H = [](double x) { return x >= 0.0 ? 1.0 : 0.0; };
                return A_ * (H(t - t0_) - 2.0 * H(t - t0_ - 0.5 * T_) + H(t - t0_ - T_));
            }
            case Kind::table:
                return table_(t);
        }
        return 0.0;
    }

    [[nodiscard]] DriverInput evaluate(double t) const { return {delta_at(t), 0.0}; }

    // §5.6 discontinuity rule: a fixed-step integrator SHALL place a step boundary
    // exactly on each discontinuity, and an adaptive one SHALL restart there.
    // Integrating across a discontinuity destroys the method order and is a
    // defect, not a tolerance issue — so the schedule must be able to say where
    // its discontinuities are.
    [[nodiscard]] std::vector<double> discontinuities() const {
        switch (kind_) {
            case Kind::step:    return {t0_};
            case Kind::doublet: return {t0_, t0_ + 0.5 * T_, t0_ + T_};
            default:            return {};
        }
    }

    [[nodiscard]] Kind kind() const { return kind_; }

private:
    Maneuver(Kind k, double A) : kind_(k), A_(A) {}
    Kind kind_;
    double A_ = 0.0;
    double t0_ = 0.0;
    double T_ = 1.0;
    numerics::Pchip table_;
};

// §5.5 model-reduction switches. Each is individually tested; together they
// define "bicycle-equivalence mode" for cross-check C-02 (§17.8).
struct ModelSwitches {
    bool load_transfer = true;
    bool aero = true;
    bool track_velocity_terms = true;  // the +/- r*t/2 of (9.14)
    bool small_angle_slip = false;     // force v/u instead of arctan(v/u), §3.3
    bool steer_projection = true;      // the cos/sin delta factors of (9.11)-(9.13)
    bool aligning_moment = true;       // include sum M_z in (9.13)
    bool longitudinal_forces = true;

    // §17.8 C-02. Not an emergent property: an explicit, tested configuration.
    [[nodiscard]] static ModelSwitches bicycle_equivalence() {
        ModelSwitches s;
        s.load_transfer = false;
        s.aero = false;
        s.track_velocity_terms = false;
        s.small_angle_slip = true;
        s.steer_projection = false;
        s.aligning_moment = false;
        s.longitudinal_forces = false;
        return s;
    }
};

struct SolverSettings {
    ModelSwitches switches{};
    SpeedMode speed_mode = SpeedMode::constant;
    double speed_floor_mps = 0.5;       // §5.5
    double small_angle_limit_rad = 0.15;
    // §9.7 algebraic loop.
    bool implicit_load_transfer = false;  // default `explicit`, matching the source
    double atol_ay = 1e-9;
    int max_ay_iterations = 20;
    double road_tilt_rad = 0.0;
};

// §4.3. State layout is shared by both plants so that C-02 can compare them
// element by element:  x = [v, r, psi, X_E, Y_E], plus a leading u under
// speed_mode = constant_accel.
namespace state {
inline constexpr int kV = 0, kR = 1, kPsi = 2, kXe = 3, kYe = 4;
inline constexpr int kDim = 5;
}  // namespace state

class Plant {
public:
    virtual ~Plant() = default;
    Plant(const Plant&) = delete;
    Plant& operator=(const Plant&) = delete;

    [[nodiscard]] virtual int state_dim() const = 0;
    virtual void derivative(double t, std::span<const double> x, const DriverInput& u,
                            std::span<double> xdot) const = 0;

protected:
    Plant() = default;
};

// (8.29) path kinematics, shared by both plants. Position states are integrated
// with the same integrator and step as the dynamics.
inline void path_kinematics(double u_mps, double v_mps, double r, double psi,
                            std::span<double> xdot) {
    xdot[state::kPsi] = r;
    xdot[state::kXe] = u_mps * std::cos(psi) - v_mps * std::sin(psi);
    xdot[state::kYe] = u_mps * std::sin(psi) + v_mps * std::cos(psi);
}

}  // namespace rcvd::models

#endif  // RCVD_MODELS_PLANT_HPP
