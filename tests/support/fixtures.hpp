// tests/support/fixtures.hpp — the reference vehicles of spec §17.2.
#ifndef RCVD_TESTS_SUPPORT_FIXTURES_HPP
#define RCVD_TESTS_SUPPORT_FIXTURES_HPP

#include "rcvd/core/vehicle.hpp"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/units/constants.hpp"

namespace rcvd::fixtures {

inline constexpr double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

// CH5 ([R280] p. 51): l = 104.5 in, W = 3700 lbf, 57 % front,
// C_F = -640 lbf/deg, C_R = -580 lbf/deg, h = 20 in.
//
// §17.2: the source does not state a yaw inertia (Ch. 5 is steady-state
// throughout); the suite's fixture adopts I_z = 1800 slug*ft^2, which affects
// only the transient vectors, never B-01 ... B-19.
inline core::VehicleInput ch5_input() {
    core::VehicleInput in;
    in.name = "CH5";
    in.weight_N = 3700.0 * units::POUND_FORCE;
    in.yaw_inertia_kg_m2 = 1800.0 * units::SLUG_FT2_TO_KG_M2;
    in.wheelbase_m = 104.5 * units::INCH;
    in.front_weight_fraction = 0.57;
    in.cg_height_m = 20.0 * units::INCH;
    in.track_front_m = 60.0 * units::INCH;
    in.track_rear_m = 60.0 * units::INCH;
    in.steering_ratio = 14.0;
    return in;
}

inline models::AxleStiffness ch5_tires() {
    return {-640.0 * kLbfDeg, -580.0 * kLbfDeg};
}

// CH5-OS: CH5 with the weight distribution reversed to 43 % front, tires
// unchanged (§8.6, §17.2). Used to bracket V_crit.
inline core::VehicleInput ch5_os_input() {
    core::VehicleInput in = ch5_input();
    in.name = "CH5-OS";
    in.front_weight_fraction = 0.43;
    return in;
}

// "First Example" ([RCVD] p. 256, [R280] p. 74): W = 2000 lbf, l = 8.33 ft,
// I_z = 832 slug*ft^2, 65 % front, C_F = -240, C_R = -200 lbf/deg, V = 40 mph.
inline core::VehicleInput first_example_input() {
    core::VehicleInput in;
    in.name = "First Example";
    in.weight_N = 2000.0 * units::POUND_FORCE;
    in.yaw_inertia_kg_m2 = 832.0 * units::SLUG_FT2_TO_KG_M2;
    in.wheelbase_m = 8.33 * units::FOOT;
    in.front_weight_fraction = 0.65;
    in.cg_height_m = 20.0 * units::INCH;
    return in;
}

inline models::AxleStiffness first_example_tires() {
    return {-240.0 * kLbfDeg, -200.0 * kLbfDeg};
}

}  // namespace rcvd::fixtures

#endif  // RCVD_TESTS_SUPPORT_FIXTURES_HPP
