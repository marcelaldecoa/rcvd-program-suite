// The mistake actually made when first writing the B-04 acceptance vector:
// displaying Y_r (N*s/rad) with the lbf*ft/deg factor (N*m/rad). Numerically it
// produced a plausible number; dimensionally it is s/m.
#include "rcvd/models/bicycle.hpp"
#include "rcvd/units/quantity.hpp"
using namespace rcvd;
using namespace rcvd::units;
int main() {
    models::StabilityDerivatives d;
    d.Y_r = 2794.5;
    return static_cast<int>(in(d.Y_r_q(), lbf_ft_per_deg));   // MUST NOT COMPILE
}
