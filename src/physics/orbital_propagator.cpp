#include "aethersat/physics/orbital_propagator.hpp"

#include <cmath>

#include "aethersat/physics/rk4.hpp"

namespace aethersat::physics {

OrbitalPropagator::OrbitalPropagator(bool j2_enabled)
    : j2_enabled_(j2_enabled) {
    const double v0 = std::sqrt(kMuEarthKm3S2 / kDefaultOrbitRadiusKm);
    state_ = {kDefaultOrbitRadiusKm, 0.0, 0.0, 0.0, v0, 0.0};
    time_s_ = 0.0;
}

OrbitalPropagator::OrbitalPropagator(const StateVector& initial_state, bool j2_enabled)
    : state_(initial_state), time_s_(0.0), j2_enabled_(j2_enabled) {}

OrbitalPropagator::StateVector OrbitalPropagator::derivative(
    const StateVector& s, double /*t*/, bool j2_enabled) noexcept {
    const double x = s[0];
    const double y = s[1];
    const double z = s[2];
    const double r2 = x * x + y * y + z * z;
    const double r = std::sqrt(r2);
    const double r3 = r2 * r;
    const double coeff = -kMuEarthKm3S2 / r3;

    double ax = coeff * x;
    double ay = coeff * y;
    double az = coeff * z;

    if (j2_enabled) {
        const double r5 = r3 * r2;
        const double z2_over_r2 = (z * z) / r2;
        const double j2_coeff = -1.5 * kJ2Earth * kMuEarthKm3S2 * (kEarthRadiusKm * kEarthRadiusKm) / r5;
        const double xy_factor = 1.0 - 5.0 * z2_over_r2;
        const double z_factor = 3.0 - 5.0 * z2_over_r2;

        ax += j2_coeff * x * xy_factor;
        ay += j2_coeff * y * xy_factor;
        az += j2_coeff * z * z_factor;
    }

    return {s[3], s[4], s[5], ax, ay, az};
}

void OrbitalPropagator::step(double dt) {
    const bool j2 = j2_enabled_;
    state_ = rk4_step<6>(state_, time_s_, dt, [j2](const StateVector& s, double t) {
        return derivative(s, t, j2);
    });
    time_s_ += dt;
}

bool OrbitalPropagator::in_eclipse() const noexcept {
    // Cylindrical Earth-shadow model with Sun vector along +X.
    // Spacecraft is in shadow when x < 0 and distance from the X-axis is < R_E.
    if (state_[0] >= 0.0) {
        return false;
    }
    const double perp_dist_sq = state_[1] * state_[1] + state_[2] * state_[2];
    return perp_dist_sq < (kEarthRadiusKm * kEarthRadiusKm);
}

double OrbitalPropagator::radius_km() const noexcept {
    return std::sqrt(state_[0] * state_[0] + state_[1] * state_[1] + state_[2] * state_[2]);
}

double OrbitalPropagator::altitude_km() const noexcept {
    return radius_km() - kEarthRadiusKm;
}

double OrbitalPropagator::speed_kms() const noexcept {
    return std::sqrt(state_[3] * state_[3] + state_[4] * state_[4] + state_[5] * state_[5]);
}

double OrbitalPropagator::specific_energy_km2_s2() const noexcept {
    const double v2 = state_[3] * state_[3] + state_[4] * state_[4] + state_[5] * state_[5];
    const double r = radius_km();
    return 0.5 * v2 - (kMuEarthKm3S2 / r);
}

double OrbitalPropagator::raan_rad() const noexcept {
    // Specific angular momentum h = r x v
    const double hx = state_[1] * state_[5] - state_[2] * state_[4];
    const double hy = state_[2] * state_[3] - state_[0] * state_[5];
    // Ascending node vector n = k_hat x h = (-hy, hx, 0)
    const double nx = -hy;
    const double ny = hx;
    return std::atan2(ny, nx);
}

void OrbitalPropagator::set_state(const StateVector& state, double time_s) noexcept {
    state_ = state;
    time_s_ = time_s;
}

}  // namespace aethersat::physics
