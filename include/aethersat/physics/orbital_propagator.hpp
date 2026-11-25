#pragma once

#include <array>

namespace aethersat::physics {

class OrbitalPropagator {
public:
    static constexpr double kMuEarthKm3S2 = 398600.4418;
    static constexpr double kEarthRadiusKm = 6378.137;
    static constexpr double kDefaultOrbitRadiusKm = 6878.137;  // ~500 km circular LEO
    static constexpr double kJ2Earth = 1.08262668e-3;

    using StateVector = std::array<double, 6>;  // [x, y, z, vx, vy, vz] in km and km/s

    explicit OrbitalPropagator(bool j2_enabled = true);
    explicit OrbitalPropagator(const StateVector& initial_state, bool j2_enabled = true);

    void step(double dt);

    const StateVector& state() const noexcept { return state_; }
    double time_s() const noexcept { return time_s_; }
    bool in_eclipse() const noexcept;
    double altitude_km() const noexcept;
    double radius_km() const noexcept;
    double speed_kms() const noexcept;
    double specific_energy_km2_s2() const noexcept;
    double raan_rad() const noexcept;

    void set_state(const StateVector& state, double time_s = 0.0) noexcept;
    void set_j2_enabled(bool enabled) noexcept { j2_enabled_ = enabled; }
    bool j2_enabled() const noexcept { return j2_enabled_; }

private:
    static StateVector derivative(const StateVector& s, double t, bool j2_enabled) noexcept;

    StateVector state_{};
    double time_s_{0.0};
    bool j2_enabled_{true};
};

}  // namespace aethersat::physics
