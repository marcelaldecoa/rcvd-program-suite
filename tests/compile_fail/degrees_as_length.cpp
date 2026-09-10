// §5.2's dimension check, at compile time rather than at load time: an angle
// cannot be read through a length unit.
#include "rcvd/units/quantity.hpp"
using namespace rcvd::units;
int main() {
    const Angle a = of(5.0, degree);
    return static_cast<int>(in(a, inch));   // MUST NOT COMPILE
}
