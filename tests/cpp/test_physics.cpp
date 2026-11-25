#include <gtest/gtest.h>

#include <cmath>

#include "aethersat/physics/attitude_adcs.hpp"
#include "aethersat/physics/orbital_propagator.hpp"
#include "aethersat/physics/power_eps.hpp"

using namespace aethersat::physics;

TEST(OrbitalPropagatorTest, SpecificOrbitalEnergyConservedOver10000Steps) {
    OrbitalPropagator prop;
    EXPECT_TRUE(prop.j2_enabled());  // J2 enabled by default
    prop.set_j2_enabled(false);      // Toggle J2 off for pure 2-body Keplerian energy conservation

    const double initial_energy = prop.specific_energy_km2_s2();

    constexpr double dt = 0.01;
    constexpr int num_steps = 10000;
    for (int i = 0; i < num_steps; ++i) {
        prop.step(dt);
    }

    const double final_energy = prop.specific_energy_km2_s2();
    const double rel_error = std::abs(final_energy - initial_energy) / std::abs(initial_energy);
    EXPECT_LT(rel_error, 1e-7) << "Relative energy drift exceeded 1e-7: " << rel_error;
}

TEST(OrbitalPropagatorTest, J2SecularNodalRegressionMatchesAnalyticalRate) {
    constexpr double kPi = 3.14159265358979323846;
    const double a = OrbitalPropagator::kDefaultOrbitRadiusKm;
    const double v0 = std::sqrt(OrbitalPropagator::kMuEarthKm3S2 / a);
    const double inc_rad = 51.6 * kPi / 180.0;  // Prograde inclined LEO (ISS-like)

    const OrbitalPropagator::StateVector init_state = {
        a, 0.0, 0.0,
        0.0, v0 * std::cos(inc_rad), v0 * std::sin(inc_rad)
    };

    const double mean_motion = std::sqrt(OrbitalPropagator::kMuEarthKm3S2 / (a * a * a));
    const double orbital_period_s = 2.0 * kPi / mean_motion;

    // Analytical secular RAAN drift rate: dOmega/dt = -1.5 * J2 * (R_E / a)^2 * n * cos(i)
    const double re_over_a = OrbitalPropagator::kEarthRadiusKm / a;
    const double expected_raan_dot =
        -1.5 * OrbitalPropagator::kJ2Earth * (re_over_a * re_over_a) * mean_motion * std::cos(inc_rad);
    const double expected_delta_raan = expected_raan_dot * orbital_period_s;

    OrbitalPropagator prop_j2(init_state, /*j2_enabled=*/true);
    OrbitalPropagator prop_kepler(init_state, /*j2_enabled=*/false);

    const double raan0_j2 = prop_j2.raan_rad();
    const double raan0_kepler = prop_kepler.raan_rad();

    constexpr int steps = 5000;
    const double dt = orbital_period_s / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
        prop_j2.step(dt);
        prop_kepler.step(dt);
    }

    const double delta_raan_j2 = prop_j2.raan_rad() - raan0_j2;
    const double delta_raan_kepler = prop_kepler.raan_rad() - raan0_kepler;

    // Westward (negative) nodal regression under J2 matching analytical secular rate within 5%
    EXPECT_LT(delta_raan_j2, 0.0);
    const double rel_err = std::abs(delta_raan_j2 - expected_delta_raan) / std::abs(expected_delta_raan);
    EXPECT_LT(rel_err, 0.05) << "Observed dRAAN=" << delta_raan_j2
                             << ", expected=" << expected_delta_raan;

    // Pure 2-body Keplerian orbit exhibits zero nodal regression
    EXPECT_NEAR(delta_raan_kepler, 0.0, 1e-9);
}

TEST(OrbitalPropagatorTest, CylindricalEclipseDetection) {
    OrbitalPropagator prop;
    // Default initial position is (+6878.137, 0, 0) -> sunlit
    EXPECT_FALSE(prop.in_eclipse());

    // Directly behind Earth along -X -> in eclipse
    prop.set_state({-6878.137, 0.0, 0.0, 0.0, -7.6, 0.0});
    EXPECT_TRUE(prop.in_eclipse());

    // Behind Earth in -X, but outside Earth shadow cylinder (y > R_E) -> sunlit
    prop.set_state({-6878.137, 6500.0, 0.0, 0.0, -7.6, 0.0});
    EXPECT_FALSE(prop.in_eclipse());
}

TEST(AttitudeAdcsTest, QuaternionNormConservedAndSlewConverges) {
    AttitudeADCS adcs;
    // 45-degree slew about normalized axis (1, 1, 1)/sqrt(3)
    const double half_angle = (45.0 * 3.14159265358979323846 / 180.0) * 0.5;
    const double s = std::sin(half_angle) / std::sqrt(3.0);
    const Quaternion target = normalize_quaternion({std::cos(half_angle), s, s, s});
    adcs.set_target_quaternion(target);

    constexpr double dt = 0.02;
    for (int i = 0; i < 1500; ++i) {
        adcs.step(dt);
        const auto& q = adcs.quaternion();
        const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        EXPECT_NEAR(norm, 1.0, 1e-9);
    }

    EXPECT_LT(adcs.pointing_error_deg(), 0.2);
}

TEST(AttitudeAdcsTest, WheelSaturationFaultHaltsControlTorque) {
    AttitudeADCS adcs;
    adcs.set_target_quaternion({0.70710678, 0.0, 0.0, 0.70710678});
    adcs.set_wheel_saturation_fault(true);

    for (int i = 0; i < 100; ++i) {
        adcs.step(0.05);
    }

    EXPECT_DOUBLE_EQ(adcs.last_torque()[0], 0.0);
    EXPECT_DOUBLE_EQ(adcs.last_torque()[1], 0.0);
    EXPECT_DOUBLE_EQ(adcs.last_torque()[2], 0.0);
    EXPECT_GT(adcs.pointing_error_deg(), 80.0);
}

TEST(PowerEpsTest, SunlightEclipseAndSafeModePowerBalance) {
    PowerEPS eps(50.0);
    const double initial_soc = eps.battery_soc_pct();
    EXPECT_NEAR(eps.bus_voltage_v(), 26.4, 1e-6);

    // Sunlight + Nominal (120W gen - 45W load = +75W net) -> charging
    eps.step(60.0, /*in_eclipse=*/false, /*safe_mode=*/false);
    EXPECT_DOUBLE_EQ(eps.solar_power_w(), 120.0);
    EXPECT_DOUBLE_EQ(eps.bus_load_w(), 45.0);
    EXPECT_GT(eps.battery_soc_pct(), initial_soc);

    const double post_charge_soc = eps.battery_soc_pct();
    // Eclipse + Safe mode (0W gen - 20W load = -20W net) -> slow discharge
    eps.step(60.0, /*in_eclipse=*/true, /*safe_mode=*/true);
    EXPECT_DOUBLE_EQ(eps.solar_power_w(), 0.0);
    EXPECT_DOUBLE_EQ(eps.bus_load_w(), 20.0);
    EXPECT_LT(eps.battery_soc_pct(), post_charge_soc);
    EXPECT_GE(eps.bus_voltage_v(), PowerEPS::kMinBusVoltageV);
    EXPECT_LE(eps.bus_voltage_v(), PowerEPS::kMaxBusVoltageV);
}
