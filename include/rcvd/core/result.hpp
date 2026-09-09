// rcvd/core/result.hpp — the error channel of spec §4.6, Rule A3.
//
// std::expected is C++23; §4.2 mandates C++20, so this is the same shape over
// std::variant with matching member names. Migrating a later revision to
// std::expected is intended to be a type alias, so do not add members that
// std::expected lacks.
#ifndef RCVD_CORE_RESULT_HPP
#define RCVD_CORE_RESULT_HPP

#include <cassert>
#include <utility>
#include <variant>

#include "rcvd/core/diagnostic.hpp"

namespace rcvd::core {

template <class T>
class Result {
public:
    Result(T value) : v_(std::move(value)) {}          // NOLINT(google-explicit-constructor)
    Result(Diagnostic d) : v_(std::move(d)) {}         // NOLINT(google-explicit-constructor)

    [[nodiscard]] bool ok() const noexcept { return v_.index() == 0; }
    explicit operator bool() const noexcept { return ok(); }

    // Precondition: ok(). §13.1 makes a silently-wrong value a defect, so this
    // asserts rather than returning a default-constructed T.
    [[nodiscard]] const T& value() const& {
        assert(ok() && "Result::value() on a failed Result");
        return std::get<0>(v_);
    }
    [[nodiscard]] T&& value() && {
        assert(ok() && "Result::value() on a failed Result");
        return std::get<0>(std::move(v_));
    }

    // Precondition: !ok().
    [[nodiscard]] const Diagnostic& error() const& {
        assert(!ok() && "Result::error() on a successful Result");
        return std::get<1>(v_);
    }

    // Convenience for call sites that have a sensible fallback. Deliberately
    // explicit at the call site: there is no implicit conversion to T.
    [[nodiscard]] T value_or(T fallback) const& { return ok() ? std::get<0>(v_) : std::move(fallback); }

private:
    std::variant<T, Diagnostic> v_;
};

// Helper so a failing routine can `return err<Foo>("E-SS-030", "...")`.
template <class T>
[[nodiscard]] Result<T> err(std::string_view code, std::string message,
                            std::optional<double> t_s = std::nullopt,
                            std::optional<Wheel> wheel = std::nullopt) {
    return Result<T>(make_diagnostic(code, std::move(message), t_s, wheel));
}

}  // namespace rcvd::core

#endif  // RCVD_CORE_RESULT_HPP
