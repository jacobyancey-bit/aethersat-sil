#pragma once

#include <array>
#include <cstddef>

namespace aethersat::physics {

/**
 * @brief Generic 4th-order Runge-Kutta (RK4) numerical integrator step.
 *
 * @tparam N State vector dimension.
 * @tparam DerivFn Callable with signature std::array<double, N>(const std::array<double, N>&, double).
 * @param state Current state vector y(t).
 * @param t Current simulation time.
 * @param dt Integration step size.
 * @param derivative_fn Right-hand-side derivative function dy/dt = f(y, t).
 * @return Integrated state vector y(t + dt).
 */
template <std::size_t N, typename DerivFn>
std::array<double, N> rk4_step(
    const std::array<double, N>& state,
    double t,
    double dt,
    DerivFn&& derivative_fn) {
    const auto k1 = derivative_fn(state, t);

    std::array<double, N> s2{};
    for (std::size_t i = 0; i < N; ++i) {
        s2[i] = state[i] + 0.5 * dt * k1[i];
    }
    const auto k2 = derivative_fn(s2, t + 0.5 * dt);

    std::array<double, N> s3{};
    for (std::size_t i = 0; i < N; ++i) {
        s3[i] = state[i] + 0.5 * dt * k2[i];
    }
    const auto k3 = derivative_fn(s3, t + 0.5 * dt);

    std::array<double, N> s4{};
    for (std::size_t i = 0; i < N; ++i) {
        s4[i] = state[i] + dt * k3[i];
    }
    const auto k4 = derivative_fn(s4, t + dt);

    std::array<double, N> next{};
    constexpr double kOneSixth = 1.0 / 6.0;
    for (std::size_t i = 0; i < N; ++i) {
        next[i] = state[i] + (dt * kOneSixth) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    return next;
}

}  // namespace aethersat::physics
