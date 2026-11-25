#include "aethersat/physics/power_eps.hpp"

#include <algorithm>

namespace aethersat::physics {

PowerEPS::PowerEPS(double initial_soc_pct) {
    set_battery_soc_pct(initial_soc_pct);
}

void PowerEPS::step(double dt, bool in_eclipse, bool safe_mode) {
    solar_power_w_ = in_eclipse ? kSolarGenEclipseW : kSolarGenSunlightW;
    bus_load_w_ = safe_mode ? kSafeModeLoadW : kNominalLoadW;

    const double net_power_w = solar_power_w_ - bus_load_w_;
    constexpr double kCapacityJoules = kBatteryCapacityWh * 3600.0;
    const double delta_soc_pct = (net_power_w * dt / kCapacityJoules) * 100.0;
    battery_soc_pct_ = std::clamp(battery_soc_pct_ + delta_soc_pct, 0.0, 100.0);
    update_bus_voltage();
}

void PowerEPS::set_battery_soc_pct(double soc_pct) noexcept {
    battery_soc_pct_ = std::clamp(soc_pct, 0.0, 100.0);
    update_bus_voltage();
}

void PowerEPS::update_bus_voltage() noexcept {
    bus_voltage_v_ = kMinBusVoltageV +
        (kMaxBusVoltageV - kMinBusVoltageV) * (battery_soc_pct_ / 100.0);
}

}  // namespace aethersat::physics
