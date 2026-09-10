// rcvd/numerics/accumulate.hpp — compensated summation, spec §13.1 and Rule A2.
//
// WHY, AND WHY NOT A WIDER TYPE. §13.1 mandates IEEE 754 binary64 everywhere, and
// that is the right call: `long double` is 80-bit on x86 System V, 64-bit under
// MSVC, and 128-bit on some AArch64 targets, so reaching for it would break
// §18.2's requirement that all platforms agree to 1e-12 — the very property it
// would appear to improve. Precision has to come from the ALGORITHM, not the type.
//
// Where it matters is long accumulations. A lap solve at 1 ms takes thousands of
// steps, and two quantities are summed once per step:
//
//   t     += h        the run clock, against §11.4's 1e-9 s event location
//   theta += dtheta   the accumulated path angle, against (11.1)'s 2*pi test
//
// Naive summation of n terms carries O(n*eps) worst-case error; Neumaier's
// compensation reduces that to O(eps), independent of n. At 30,000 steps that is
// the difference between ~7e-12 and ~2e-16 relative — small, but (11.1) is
// compared against 2*pi and §11.4 asks for 1e-9 s, so it is not free margin.
//
// Rule A2 is preserved: the operation order is fixed and deterministic, so the
// result is bit-reproducible run to run.
#ifndef RCVD_NUMERICS_ACCUMULATE_HPP
#define RCVD_NUMERICS_ACCUMULATE_HPP

#include <cmath>
#include <span>

namespace rcvd::numerics {

// Neumaier's variant of Kahan summation. Unlike plain Kahan it is also correct
// when the incoming term is LARGER than the running sum, which happens on the
// first step of a run and after any step-size increase.
class CompensatedSum {
public:
    constexpr CompensatedSum() = default;
    explicit constexpr CompensatedSum(double initial) : sum_(initial) {}

    constexpr void add(double v) {
        // Not std::abs: it is not constexpr under MSVC, and this must stay usable
        // in a constant expression.
        const auto mag = [](double x) { return x < 0.0 ? -x : x; };
        const double t = sum_ + v;
        // The lost low-order bits go to the compensation term, whichever operand
        // dominated the addition.
        if (mag(sum_) >= mag(v)) {
            c_ += (sum_ - t) + v;
        } else {
            c_ += (v - t) + sum_;
        }
        sum_ = t;
    }

    constexpr CompensatedSum& operator+=(double v) {
        add(v);
        return *this;
    }

    // The compensation is applied only on read, so `value()` is the accurate sum
    // while `sum_` alone stays a plain running total.
    [[nodiscard]] constexpr double value() const { return sum_ + c_; }
    [[nodiscard]] constexpr double uncompensated() const { return sum_; }
    [[nodiscard]] constexpr double compensation() const { return c_; }

    constexpr void reset(double initial = 0.0) {
        sum_ = initial;
        c_ = 0.0;
    }

private:
    double sum_ = 0.0;
    double c_ = 0.0;
};

// A fixed-order compensated reduction. Rule A2 forbids parallel reduction over
// floating point "without a fixed reduction tree"; this is the sequential tree,
// and it is what any parallel version must reproduce bit for bit.
[[nodiscard]] inline double compensated_sum(std::span<const double> xs) {
    CompensatedSum s;
    for (double x : xs) s.add(x);
    return s.value();
}

}  // namespace rcvd::numerics

#endif  // RCVD_NUMERICS_ACCUMULATE_HPP
