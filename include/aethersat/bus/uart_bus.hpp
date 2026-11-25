#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "aethersat/bus/virtual_bus.hpp"

namespace aethersat::bus {

class UartBus : public VirtualBus {
public:
    static constexpr uint8_t kSyncByte0 = 0xAA;
    static constexpr uint8_t kSyncByte1 = 0x55;
    static constexpr uint32_t kBaud9600 = 9600;
    static constexpr uint32_t kBaud115200 = 115200;

    explicit UartBus(uint32_t baud_rate = kBaud115200);

    std::string name() const override { return "UART-StarTracker"; }
    void reset() override;
    BusStats stats() const noexcept override { return stats_; }

    uint32_t baud_rate() const noexcept { return baud_rate_; }
    bool set_baud_rate(uint32_t baud_rate) noexcept;

    // Frame format: [0xAA, 0x55, payload_len (u8), payload..., xor_checksum (u8)]
    static uint8_t compute_checksum(uint8_t length, const std::vector<uint8_t>& payload) noexcept;
    std::vector<uint8_t> encode_frame(const std::vector<uint8_t>& payload);
    std::optional<std::vector<uint8_t>> decode_frame(const std::vector<uint8_t>& raw_frame);

    // Star Tracker attitude quaternion helpers (4x big-endian float32 = 16 bytes)
    std::vector<uint8_t> encode_star_tracker_quaternion(const std::array<double, 4>& q);
    std::optional<std::array<double, 4>> decode_star_tracker_quaternion(
        const std::vector<uint8_t>& raw_frame);

private:
    uint32_t baud_rate_{kBaud115200};
    BusStats stats_{};
};

}  // namespace aethersat::bus
