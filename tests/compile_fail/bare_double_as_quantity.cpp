// A bare double is not a quantity: the constructor is explicit, so an unconverted
// number cannot drift into a typed interface.
#include "rcvd/units/quantity.hpp"
using namespace rcvd::units;
int main() {
    const Force f = 1000.0;   // MUST NOT COMPILE
    return static_cast<int>(f.si());
}
