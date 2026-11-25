#pragma once

#include <array>

namespace aethersat::physics {

using Quaternion = std::array<double, 4>;  // [qw, qx, qy, qz]
using Vec3 = std::array<double, 3>;        // [x, y, z]

Quaternion normalize_quaternion(const Quaternion& q) noexcept;
Quaternion quaternion_conjugate(const Quaternion& q) noexcept;
Quaternion quaternion_multiply(const Quaternion& a, const Quaternion& b) noexcept;
double quaternion_error_angle_rad(const Quaternion& q, const Quaternion& target) noexcept;

class AttitudeADCS {
public:
    static constexpr Vec3 kInertiaDiag = {12.0, 12.0, 8.0};  // kg*m^2
    static constexpr double kMaxWheelTorqueNm = 0.5;
    static constexpr double kMaxWheelMomentumNms = 10.0;
    static constexpr double kDefaultKp = 6.0;
    static constexpr double kDefaultKd = 14.0;

    AttitudeADCS();

    // Computes PD control torque toward q_target using measured attitude & current omega.
    Vec3 compute_control_torque(const Quaternion& measured_q) const noexcept;
    Vec3 compute_control_torque() const noexcept { return compute_control_torque(q_); }

    // Propagates attitude kinematics and rigid-body dynamics under applied reaction-wheel torque.
    void step(double dt, const Vec3& commanded_torque_nm);
    // Convenience step using internal PD controller.
    void step(double dt);

    void set_target_quaternion(const Quaternion& q_target) noexcept;
    void set_attitude(const Quaternion& q, const Vec3& omega) noexcept;
    void set_wheel_saturation_fault(bool enabled) noexcept { wheel_saturation_fault_ = enabled; }
    void set_gains(double kp, double kd) noexcept { kp_ = kp; kd_ = kd; }

    const Quaternion& quaternion() const noexcept { return q_; }
    const Quaternion& target_quaternion() const noexcept { return q_target_; }
    const Vec3& angular_velocity() const noexcept { return omega_; }
    const Vec3& last_torque() const noexcept { return last_torque_nm_; }
    const Vec3& wheel_momentum() const noexcept { return wheel_momentum_nms_; }
    bool wheel_saturation_fault() const noexcept { return wheel_saturation_fault_; }
    double pointing_error_deg() const noexcept;

private:
    Quaternion q_{1.0, 0.0, 0.0, 0.0};
    Quaternion q_target_{1.0, 0.0, 0.0, 0.0};
    Vec3 omega_{0.0, 0.0, 0.0};
    Vec3 last_torque_nm_{0.0, 0.0, 0.0};
    Vec3 wheel_momentum_nms_{0.0, 0.0, 0.0};
    double kp_{kDefaultKp};
    double kd_{kDefaultKd};
    double time_s_{0.0};
    bool wheel_saturation_fault_{false};
};

}  // namespace aethersat::physics
