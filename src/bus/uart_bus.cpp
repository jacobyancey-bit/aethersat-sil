#include "aethersat/bus/uart_bus.hpp"

#include <cstring>
#include <stdexcept>

namespace aethersat::bus {

namespace {

void append_f32_be(std::vector<uint8_t>& buf, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    buf.push_back(static_cast<uint8_t>((bits >> 24U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>((bits >> 16U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>((bits >> 8U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>(bits & 0xFFU));
}

float read_f32_be(const uint8_t* ptr) {
    const uint32_t bits =
        (static_cast<uint32_t>(ptr[0]) << 24U) |
        (static_cast<uint32_t>(ptr[1]) << 16U) |
        (static_cast<uint32_t>(ptr[2]) << 8U) |
        static_cast<uint32_t>(ptr[3]);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

UartBus::UartBus(uint32_t baud_rate) {
    if (!set_baud_rate(baud_rate)) {
        baud_rate_ = kBaud115200;
    }
}

void UartBus::reset() {
    stats_ = {};
}

bool UartBus::set_baud_rate(uint32_t baud_rate) noexcept {
    if (baud_rate != kBaud9600 && baud_rate != kBaud115200) {
        return false;
    }
    baud_rate_ = baud_rate;
    return true;
}

uint8_t UartBus::compute_checksum(uint8_t length, const std::vector<uint8_t>& payload) noexcept {
    uint8_t csum = length;
    for (uint8_t b : payload) {
        csum ^= b;
    }
    return csum;
}

std::vector<uint8_t> UartBus::encode_frame(const std::vector<uint8_t>& payload) {
    if (payload.size() > 255U) {
        throw std::invalid_argument("UART payload exceeds 255 bytes");
    }
    const uint8_t len = static_cast<uint8_t>(payload.size());
    const uint8_t csum = compute_checksum(len, payload);

    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 4U);
    frame.push_back(kSyncByte0);
    frame.push_back(kSyncByte1);
    frame.push_back(len);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(csum);

    ++stats_.tx_frames;
    return frame;
}

std::optional<std::vector<uint8_t>> UartBus::decode_frame(const std::vector<uint8_t>& raw_frame) {
    if (raw_frame.size() < 4U) {
        ++stats_.error_count;
        return std::nullopt;
    }
    if (raw_frame[0] != kSyncByte0 || raw_frame[1] != kSyncByte1) {
        ++stats_.error_count;
        return std::nullopt;
    }
    const uint8_t len = raw_frame[2];
    if (raw_frame.size() != static_cast<std::size_t>(len) + 4U) {
        ++stats_.error_count;
        return std::nullopt;
    }

    std::vector<uint8_t> payload(raw_frame.begin() + 3, raw_frame.begin() + 3 + len);
    const uint8_t expected_csum = compute_checksum(len, payload);
    const uint8_t received_csum = raw_frame.back();
    if (expected_csum != received_csum) {
        ++stats_.error_count;
        return std::nullopt;
    }

    ++stats_.rx_frames;
    return payload;
}

std::vector<uint8_t> UartBus::encode_star_tracker_quaternion(const std::array<double, 4>& q) {
    std::vector<uint8_t> payload;
    payload.reserve(16U);
    for (double component : q) {
        append_f32_be(payload, static_cast<float>(component));
    }
    return encode_frame(payload);
}

std::optional<std::array<double, 4>> UartBus::decode_star_tracker_quaternion(
    const std::vector<uint8_t>& raw_frame) {
    const auto payload = decode_frame(raw_frame);
    if (!payload.has_value() || payload->size() != 16U) {
        if (payload.has_value()) {
            ++stats_.error_count;
        }
        return std::nullopt;
    }

    std::array<double, 4> q{};
    for (std::size_t i = 0; i < 4U; ++i) {
        q[i] = static_cast<double>(read_f32_be(payload->data() + i * 4U));
    }
    return q;
}

}  // namespace aethersat::bus
