// The deg<->rad roll-stiffness error that §9.5's I-3c exists to catch, and which
// neither I-3 nor I-3b can see: reading a roll stiffness through the cornering
// stiffness unit.
#include "rcvd/units/quantity.hpp"
using namespace rcvd::units;
int main() {
    const AngularStiffness k = of(975.0, lbf_ft_per_deg);
    return static_cast<int>(in(k, lbf_per_deg));   // MUST NOT COMPILE
}
