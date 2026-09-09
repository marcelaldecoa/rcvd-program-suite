// rcvd/numerics/linalg.hpp — LU with partial pivoting and rcond, spec §13.8.
//
// §8.5 is normative: the 2x2 steady-state system is solved by LU, never by an
// explicit inverse, and the reciprocal condition number is checked so that a
// vehicle at or above its critical speed returns E-SS-030 rather than a huge
// number. Every solve returns rcond (§13.8).
#ifndef RCVD_NUMERICS_LINALG_HPP
#define RCVD_NUMERICS_LINALG_HPP

#include <array>
#include <cmath>
#include <cstddef>

namespace rcvd::numerics {

template <std::size_t N>
struct LuSolution {
    std::array<double, N> x{};
    double rcond{0.0};   // 1/(||A||_inf * ||A^-1||_inf), estimated
    bool   singular{false};
};

namespace detail {

template <std::size_t N>
[[nodiscard]] inline double norm_inf(const std::array<std::array<double, N>, N>& A) noexcept {
    double n = 0.0;
    for (std::size_t i = 0; i < N; ++i) {
        double row = 0.0;
        for (std::size_t j = 0; j < N; ++j) row += std::abs(A[i][j]);
        n = std::max(n, row);
    }
    return n;
}

// Factor in place; returns false on an exactly singular pivot.
template <std::size_t N>
[[nodiscard]] inline bool lu_factor(std::array<std::array<double, N>, N>& A,
                                    std::array<std::size_t, N>& piv) noexcept {
    for (std::size_t i = 0; i < N; ++i) piv[i] = i;
    for (std::size_t k = 0; k < N; ++k) {
        std::size_t p = k;
        double maxv = std::abs(A[k][k]);
        for (std::size_t i = k + 1; i < N; ++i) {
            if (std::abs(A[i][k]) > maxv) { maxv = std::abs(A[i][k]); p = i; }
        }
        if (maxv == 0.0) return false;
        if (p != k) { std::swap(A[p], A[k]); std::swap(piv[p], piv[k]); }
        for (std::size_t i = k + 1; i < N; ++i) {
            A[i][k] /= A[k][k];
            for (std::size_t j = k + 1; j < N; ++j) A[i][j] -= A[i][k] * A[k][j];
        }
    }
    return true;
}

template <std::size_t N>
inline void lu_solve_inplace(const std::array<std::array<double, N>, N>& LU,
                             const std::array<std::size_t, N>& piv,
                             std::array<double, N>& b) noexcept {
    std::array<double, N> y{};
    for (std::size_t i = 0; i < N; ++i) {
        double s = b[piv[i]];
        for (std::size_t j = 0; j < i; ++j) s -= LU[i][j] * y[j];
        y[i] = s;
    }
    for (std::size_t ii = N; ii-- > 0;) {
        double s = y[ii];
        for (std::size_t j = ii + 1; j < N; ++j) s -= LU[ii][j] * b[j];
        b[ii] = s / LU[ii][ii];
    }
}

}  // namespace detail

// Solve A x = b. rcond is estimated by forming A^-1 column by column through the
// same factorisation — affordable at N = 2 and N = 4, which is all this suite uses.
template <std::size_t N>
[[nodiscard]] inline LuSolution<N> lu_solve(std::array<std::array<double, N>, N> A,
                                            std::array<double, N> b) noexcept {
    LuSolution<N> out;
    const double anorm = detail::norm_inf<N>(A);

    std::array<std::size_t, N> piv{};
    if (!detail::lu_factor<N>(A, piv)) {
        out.singular = true;
        out.rcond = 0.0;
        return out;
    }

    std::array<double, N> x = b;
    detail::lu_solve_inplace<N>(A, piv, x);
    out.x = x;

    std::array<std::array<double, N>, N> inv{};
    for (std::size_t c = 0; c < N; ++c) {
        std::array<double, N> e{};
        e[c] = 1.0;
        detail::lu_solve_inplace<N>(A, piv, e);
        for (std::size_t r = 0; r < N; ++r) inv[r][c] = e[r];
    }
    const double inorm = detail::norm_inf<N>(inv);
    out.rcond = (anorm > 0.0 && inorm > 0.0) ? 1.0 / (anorm * inorm) : 0.0;
    return out;
}

// §8.6: eigenvalues of the 2x2 bicycle-model A by closed-form discriminant, not a
// general QR routine, with the trace/determinant cross-check left to the caller.
struct Eig2 {
    double re1, im1, re2, im2;
    [[nodiscard]] bool complex_pair() const noexcept { return im1 != 0.0; }
};

[[nodiscard]] inline Eig2 eig2x2(const std::array<std::array<double, 2>, 2>& A) noexcept {
    const double tr  = A[0][0] + A[1][1];
    const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    const double disc = tr * tr - 4.0 * det;
    if (disc >= 0.0) {
        const double s = std::sqrt(disc);
        return Eig2{0.5 * (tr + s), 0.0, 0.5 * (tr - s), 0.0};
    }
    const double s = std::sqrt(-disc);
    return Eig2{0.5 * tr, 0.5 * s, 0.5 * tr, -0.5 * s};
}

}  // namespace rcvd::numerics

#endif  // RCVD_NUMERICS_LINALG_HPP
