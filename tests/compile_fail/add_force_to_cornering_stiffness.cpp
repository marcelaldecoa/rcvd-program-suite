// Rule U6's literal example: "Newton_per_radian + Newton fails to compile".
#include "rcvd/units/quantity.hpp"
using namespace rcvd::units;
int main() {
    const CorneringStiffness c = of(-640.0, lbf_per_deg);
    const Force f = of(1000.0, newton);
    const auto bad = c + f;   // MUST NOT COMPILE
    return static_cast<int>(bad.si());
}
