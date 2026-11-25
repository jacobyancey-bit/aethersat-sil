#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aethersat/bus/can_bus.hpp"
#include "aethersat/bus/mil_std_1553.hpp"
#include "aethersat/bus/uart_bus.hpp"
#include "aethersat/physics/attitude_adcs.hpp"
#include "aethersat/physics/orbital_propagator.hpp"
#include "aethersat/physics/power_eps.hpp"

namespace aethersat::sim {

enum class SpacecraftMode : uint8_t {
    Nominal = 0,
    SafeMode = 1
};

enum class FaultType : uint8_t {
    None = 0,
    WheelSaturation = 1,
    Bus1553Timeout = 2
};

constexpr uint16_t kApidAdcsOrbitTm = 0x100;
constexpr uint16_t kApidEpsHealthTm = 0x200;
constexpr uint16_t kApidTelecommand = 0x300;
constexpr uint16_t kApidTcAck       = 0x301;

constexpr uint8_t kCmdSetAttitude = 0x01;
constexpr uint8_t kCmdSetMode     = 0x02;
constexpr uint8_t kCmdInjectFault = 0x03;

// Thin RAII wrappers around C <zmq.h> (no cppzmq header dependency required)
class ZmqContext {
public:
    ZmqContext();
    ~ZmqContext();

    ZmqContext(const ZmqContext&) = delete;
    ZmqContext& operator=(const ZmqContext&) = delete;

    void* raw() const noexcept { return ctx_; }

private:
    void* ctx_{nullptr};
};

class ZmqSocket {
public:
    ZmqSocket(const ZmqContext& ctx, int type);
    ~ZmqSocket();

    ZmqSocket(const ZmqSocket&) = delete;
    ZmqSocket& operator=(const ZmqSocket&) = delete;

    void bind(const std::string& endpoint);
    bool send(const std::vector<uint8_t>& data, int flags = 0);
    bool recv(std::vector<uint8_t>& out, int flags = 0);

    void* raw() const noexcept { return sock_; }

private:
    void* sock_{nullptr};
};

class SpacecraftSim {
public:
    SpacecraftSim();

    // Advance simulation by dt seconds (internally sub-steps if dt > kMaxSubStepS)
    void step(double dt);

    // Handle an incoming binary CCSDS Telecommand frame; returns binary CCSDS ACK/NACK frame
    std::vector<uint8_t> handle_telecommand(const std::vector<uint8_t>& raw_tc);

    // Build binary CCSDS Telemetry frames
    std::vector<uint8_t> build_adcs_orbit_tm();
    std::vector<uint8_t> build_eps_health_tm();

    // Accessors
    double sim_time_s() const noexcept { return sim_time_s_; }
    SpacecraftMode mode() const noexcept { return mode_; }
    FaultType active_fault() const noexcept { return active_fault_; }
    bool last_1553_error() const noexcept { return last_1553_error_; }
    uint16_t last_1553_status_word() const noexcept { return last_1553_status_word_; }

    const physics::OrbitalPropagator& orbit() const noexcept { return orbit_; }
    const physics::AttitudeADCS& adcs() const noexcept { return adcs_; }
    const physics::PowerEPS& eps() const noexcept { return eps_; }

    void set_mode(SpacecraftMode mode) noexcept { mode_ = mode; }
    void inject_fault(FaultType fault) noexcept;

private:
    static constexpr double kMaxSubStepS = 0.05;

    void step_once(double dt);
    std::vector<uint8_t> build_tc_ack(uint8_t opcode, uint8_t status);

    physics::OrbitalPropagator orbit_{};
    physics::AttitudeADCS adcs_{};
    physics::PowerEPS eps_{};

    bus::UartBus star_tracker_uart_{bus::UartBus::kBaud115200};
    bus::CanBus rw_can_{};
    bus::MilStd1553Bus eps_1553_{};

    double sim_time_s_{0.0};
    SpacecraftMode mode_{SpacecraftMode::Nominal};
    FaultType active_fault_{FaultType::None};

    bool star_tracker_valid_{true};
    bool last_1553_error_{false};
    uint16_t last_1553_status_word_{0};
    std::array<double, 3> last_rw_momentum_nms_{0.0, 0.0, 0.0};

    uint16_t seq_adcs_{0};
    uint16_t seq_eps_{0};
    uint16_t seq_ack_{0};
};

}  // namespace aethersat::sim
