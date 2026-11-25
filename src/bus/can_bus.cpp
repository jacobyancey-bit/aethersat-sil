#include "aethersat/bus/can_bus.hpp"

#include <algorithm>
#include <cmath>

namespace aethersat::bus {

namespace {

int16_t clamp_i16(double scaled) noexcept {
    const double clamped = std::clamp(scaled, -32767.0, 32767.0);
    return static_cast<int16_t>(std::llround(clamped));
}

void write_i16_be(std::array<uint8_t, 8>& buf, std::size_t offset, int16_t value) noexcept {
    const uint16_t raw = static_cast<uint16_t>(value);
    buf[offset] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
    buf[offset + 1U] = static_cast<uint8_t>(raw & 0xFFU);
}

int16_t read_i16_be(const std::array<uint8_t, 8>& buf, std::size_t offset) noexcept {
    const uint16_t raw =
        static_cast<uint16_t>((static_cast<uint16_t>(buf[offset]) << 8U) | buf[offset + 1U]);
    return static_cast<int16_t>(raw);
}

}  // namespace

void CanBus::reset() {
    while (!mailbox_.empty()) {
        mailbox_.pop();
    }
    stats_ = {};
}

bool CanBus::transmit(const CanFrame& frame) {
    if (frame.id > kMaxStandardId || frame.dlc > 8U) {
        ++stats_.error_count;
        return false;
    }
    mailbox_.push(frame);
    ++stats_.tx_frames;
    return true;
}

std::optional<CanFrame> CanBus::receive() {
    if (mailbox_.empty()) {
        return std::nullopt;
    }
    CanFrame frame = mailbox_.front();
    mailbox_.pop();
    ++stats_.rx_frames;
    return frame;
}

uint8_t CanBus::compute_frame_checksum(const std::array<uint8_t, 8>& data) noexcept {
    uint8_t csum = 0;
    for (std::size_t i = 0; i < 7U; ++i) {
        csum ^= data[i];
    }
    return csum;
}

CanFrame CanBus::pack_rw_torque_cmd(const std::array<double, 3>& torque_nm, uint8_t flags) {
    CanFrame frame{};
    frame.id = kReactionWheelCmdId;
    frame.dlc = 8U;
    // Scale: 1 LSB = 0.0001 N*m (range +/- 3.2767 N*m)
    write_i16_be(frame.data, 0U, clamp_i16(torque_nm[0] * 10000.0));
    write_i16_be(frame.data, 2U, clamp_i16(torque_nm[1] * 10000.0));
    write_i16_be(frame.data, 4U, clamp_i16(torque_nm[2] * 10000.0));
    frame.data[6] = flags;
    frame.data[7] = compute_frame_checksum(frame.data);
    return frame;
}

std::optional<std::array<double, 3>> CanBus::unpack_rw_torque_cmd(const CanFrame& frame) {
    if (frame.id != kReactionWheelCmdId || frame.dlc != 8U) {
        return std::nullopt;
    }
    if (frame.data[7] != compute_frame_checksum(frame.data)) {
        return std::nullopt;
    }
    return std::array<double, 3>{
        static_cast<double>(read_i16_be(frame.data, 0U)) * 1e-4,
        static_cast<double>(read_i16_be(frame.data, 2U)) * 1e-4,
        static_cast<double>(read_i16_be(frame.data, 4U)) * 1e-4
    };
}

CanFrame CanBus::pack_rw_momentum_tlm(const std::array<double, 3>& momentum_nms, uint8_t status_flags) {
    CanFrame frame{};
    frame.id = kReactionWheelTlmId;
    frame.dlc = 8U;
    // Scale: 1 LSB = 0.001 N*m*s (range +/- 32.767 N*m*s)
    write_i16_be(frame.data, 0U, clamp_i16(momentum_nms[0] * 1000.0));
    write_i16_be(frame.data, 2U, clamp_i16(momentum_nms[1] * 1000.0));
    write_i16_be(frame.data, 4U, clamp_i16(momentum_nms[2] * 1000.0));
    frame.data[6] = status_flags;
    frame.data[7] = compute_frame_checksum(frame.data);
    return frame;
}

std::optional<std::array<double, 3>> CanBus::unpack_rw_momentum_tlm(const CanFrame& frame) {
    if (frame.id != kReactionWheelTlmId || frame.dlc != 8U) {
        return std::nullopt;
    }
    if (frame.data[7] != compute_frame_checksum(frame.data)) {
        return std::nullopt;
    }
    return std::array<double, 3>{
        static_cast<double>(read_i16_be(frame.data, 0U)) * 1e-3,
        static_cast<double>(read_i16_be(frame.data, 2U)) * 1e-3,
        static_cast<double>(read_i16_be(frame.data, 4U)) * 1e-3
    };
}

}  // namespace aethersat::bus
