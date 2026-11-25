#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <queue>

#include "aethersat/bus/virtual_bus.hpp"

namespace aethersat::bus {

struct CanFrame {
    uint16_t id{0};                  // 11-bit CAN 2.0A Standard ID (0x000..0x7FF)
    uint8_t dlc{8};                  // Data Length Code (0..8)
    std::array<uint8_t, 8> data{};
};

class CanBus : public VirtualBus {
public:
    static constexpr uint16_t kMaxStandardId = 0x7FF;
    static constexpr uint16_t kReactionWheelCmdId = 0x101;
    static constexpr uint16_t kReactionWheelTlmId = 0x201;

    CanBus() = default;

    std::string name() const override { return "CAN-2.0A-ReactionWheels"; }
    void reset() override;
    BusStats stats() const noexcept override { return stats_; }

    bool transmit(const CanFrame& frame);
    std::optional<CanFrame> receive();

    // Pack/unpack 3-axis torque (N*m, scale 1e-4) or momentum (N*m*s, scale 1e-3) into 8-byte CAN frame
    static CanFrame pack_rw_torque_cmd(const std::array<double, 3>& torque_nm, uint8_t flags = 0);
    static std::optional<std::array<double, 3>> unpack_rw_torque_cmd(const CanFrame& frame);

    static CanFrame pack_rw_momentum_tlm(const std::array<double, 3>& momentum_nms, uint8_t status_flags = 0);
    static std::optional<std::array<double, 3>> unpack_rw_momentum_tlm(const CanFrame& frame);

private:
    static uint8_t compute_frame_checksum(const std::array<uint8_t, 8>& data) noexcept;

    std::queue<CanFrame> mailbox_;
    BusStats stats_{};
};

}  // namespace aethersat::bus
