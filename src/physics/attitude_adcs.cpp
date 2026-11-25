#include "aethersat/physics/attitude_adcs.hpp"

#include <algorithm>
#include <cmath>

#include "aethersat/physics/rk4.hpp"

namespace aethersat::physics {

namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}

Quaternion normalize_quaternion(const Quaternion& q) noexcept {
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm <= 1e-15) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    const double inv = 1.0 / norm;
    return {q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv};
}

Quaternion quaternion_conjugate(const Quaternion& q) noexcept {
    return {q[0], -q[1], -q[2], -q[3]};
}

Quaternion quaternion_multiply(const Quaternion& a, const Quaternion& b) noexcept {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    };
}

double quaternion_error_angle_rad(const Quaternion& q, const Quaternion& target) noexcept {
    const double dot = std::abs(
        q[0] * target[0] + q[1] * target[1] + q[2] * target[2] + q[3] * target[3]);
    const double clamped = std::clamp(dot, 0.0, 1.0);
    return 2.0 * std::acos(clamped);
}

AttitudeADCS::AttitudeADCS() = default;

Vec3 AttitudeADCS::compute_control_torque(const Quaternion& measured_q) const noexcept {
    const Quaternion q_norm = normalize_quaternion(measured_q);
    Quaternion q_err = quaternion_multiply(quaternion_conjugate(q_norm), q_target_);
    // Shortest-path rotation: ensure scalar part >= 0
    if (q_err[0] < 0.0) {
        q_err[0] = -q_err[0];
        q_err[1] = -q_err[1];
        q_err[2] = -q_err[2];
        q_err[3] = -q_err[3];
    }

    Vec3 tau{};
    for (std::size_t i = 0; i < 3; ++i) {
        const double raw_tau = kp_ * q_err[i + 1] - kd_ * omega_[i];
        tau[i] = std::clamp(raw_tau, -kMaxWheelTorqueNm, kMaxWheelTorqueNm);
    }
    return tau;
}

void AttitudeADCS::step(double dt, const Vec3& commanded_torque_nm) {
    Vec3 applied_torque = {0.0, 0.0, 0.0};
    if (!wheel_saturation_fault_) {
        for (std::size_t i = 0; i < 3; ++i) {
            applied_torque[i] = std::clamp(commanded_torque_nm[i], -kMaxWheelTorqueNm, kMaxWheelTorqueNm);
            wheel_momentum_nms_[i] = std::clamp(
                wheel_momentum_nms_[i] + applied_torque[i] * dt,
                -kMaxWheelMomentumNms,
                kMaxWheelMomentumNms);
        }
    } else {
        // Wheels saturated: momentum pinned at saturation limit, zero net control torque
        wheel_momentum_nms_ = {kMaxWheelMomentumNms, kMaxWheelMomentumNms, kMaxWheelMomentumNms};
    }
    last_torque_nm_ = applied_torque;

    std::array<double, 7> state = {
        q_[0], q_[1], q_[2], q_[3],
        omega_[0], omega_[1], omega_[2]
    };

    const auto deriv = [applied_torque](const std::array<double, 7>& s, double /*t*/) {
        const double qw = s[0], qx = s[1], qy = s[2], qz = s[3];
        const double wx = s[4], wy = s[5], wz = s[6];

        // dq/dt = 0.5 * Omega(omega) * q
        const double dqw = 0.5 * (-qx * wx - qy * wy - qz * wz);
        const double dqx = 0.5 * ( qw * wx + qy * wz - qz * wy);
        const double dqy = 0.5 * ( qw * wy - qx * wz + qz * wx);
        const double dqz = 0.5 * ( qw * wz + qx * wy - qy * wx);

        // I * d(omega)/dt = tau - omega x (I * omega)
        constexpr double Ix = kInertiaDiag[0];
        constexpr double Iy = kInertiaDiag[1];
        constexpr double Iz = kInertiaDiag[2];

        const double dwx = (applied_torque[0] - (Iz - Iy) * wy * wz) / Ix;
        const double dwy = (applied_torque[1] - (Ix - Iz) * wz * wx) / Iy;
        const double dwz = (applied_torque[2] - (Iy - Ix) * wx * wy) / Iz;

        return std::array<double, 7>{dqw, dqx, dqy, dqz, dwx, dwy, dwz};
    };

    const auto next = rk4_step<7>(state, time_s_, dt, deriv);
    q_ = normalize_quaternion({next[0], next[1], next[2], next[3]});
    omega_ = {next[4], next[5], next[6]};
    time_s_ += dt;
}

void AttitudeADCS::step(double dt) {
    step(dt, compute_control_torque());
}

void AttitudeADCS::set_target_quaternion(const Quaternion& q_target) noexcept {
    q_target_ = normalize_quaternion(q_target);
}

void AttitudeADCS::set_attitude(const Quaternion& q, const Vec3& omega) noexcept {
    q_ = normalize_quaternion(q);
    omega_ = omega;
}

double AttitudeADCS::pointing_error_deg() const noexcept {
    return quaternion_error_angle_rad(q_, q_target_) * kRadToDeg;
}

}  // namespace aethersat::physics
