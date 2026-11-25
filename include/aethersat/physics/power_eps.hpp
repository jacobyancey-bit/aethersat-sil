#pragma once

namespace aethersat::physics {

class PowerEPS {
public:
    static constexpr double kSolarGenSunlightW = 120.0;
    static constexpr double kSolarGenEclipseW = 0.0;
    static constexpr double kNominalLoadW = 45.0;
    static constexpr double kSafeModeLoadW = 20.0;
    static constexpr double kMinBusVoltageV = 24.0;
    static constexpr double kMaxBusVoltageV = 28.8;
    static constexpr double kBatteryCapacityWh = 100.0;

    explicit PowerEPS(double initial_soc_pct = 80.0);

    void step(double dt, bool in_eclipse, bool safe_mode);

    double solar_power_w() const noexcept { return solar_power_w_; }
    double bus_load_w() const noexcept { return bus_load_w_; }
    double battery_soc_pct() const noexcept { return battery_soc_pct_; }
    double bus_voltage_v() const noexcept { return bus_voltage_v_; }

    void set_battery_soc_pct(double soc_pct) noexcept;

private:
    void update_bus_voltage() noexcept;

    double solar_power_w_{kSolarGenSunlightW};
    double bus_load_w_{kNominalLoadW};
    double battery_soc_pct_{80.0};
    double bus_voltage_v_{27.84};
};

}  // namespace aethersat::physics
