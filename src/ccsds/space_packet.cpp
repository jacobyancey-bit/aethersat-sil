#include "aethersat/ccsds/space_packet.hpp"

#include <stdexcept>

namespace aethersat::ccsds {

uint16_t crc16_ccitt(const uint8_t* data, std::size_t length) noexcept {
    uint16_t crc = kCrc16CcittInit;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ kCrc16CcittPoly);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

std::vector<uint8_t> encode(const SpacePacket& packet) {
    if (packet.payload.size() > 65534U) {
        throw std::invalid_argument("CCSDS payload exceeds maximum 16-bit data length");
    }

    const uint16_t data_length = static_cast<uint16_t>(packet.payload.size() + kCrcBytes - 1U);

    const uint16_t word0 =
        static_cast<uint16_t>((static_cast<uint16_t>(packet.header.version & 0x07U) << 13U) |
                              (static_cast<uint16_t>(static_cast<uint8_t>(packet.header.type) & 0x01U) << 12U) |
                              (static_cast<uint16_t>(packet.header.secondary_header_flag ? 1U : 0U) << 11U) |
                              (packet.header.apid & kMaxApid));

    const uint16_t word1 =
        static_cast<uint16_t>((static_cast<uint16_t>(static_cast<uint8_t>(packet.header.sequence_flags) & 0x03U) << 14U) |
                              (packet.header.sequence_count & kMaxSeqCount));

    std::vector<uint8_t> frame;
    frame.reserve(kPrimaryHeaderBytes + packet.payload.size() + kCrcBytes);

    // Big-endian 6-byte CCSDS Primary Header
    frame.push_back(static_cast<uint8_t>((word0 >> 8U) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(word0 & 0xFFU));
    frame.push_back(static_cast<uint8_t>((word1 >> 8U) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(word1 & 0xFFU));
    frame.push_back(static_cast<uint8_t>((data_length >> 8U) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(data_length & 0xFFU));

    // Payload
    frame.insert(frame.end(), packet.payload.begin(), packet.payload.end());

    // CRC-16-CCITT over header + payload
    const uint16_t crc = crc16_ccitt(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 8U) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));

    return frame;
}

std::optional<SpacePacket> decode(const uint8_t* data, std::size_t length) {
    if (data == nullptr || length < kPrimaryHeaderBytes + kCrcBytes) {
        return std::nullopt;
    }

    const uint16_t word0 =
        static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
    const uint16_t word1 =
        static_cast<uint16_t>((static_cast<uint16_t>(data[2]) << 8U) | data[3]);
    const uint16_t data_length =
        static_cast<uint16_t>((static_cast<uint16_t>(data[4]) << 8U) | data[5]);

    const uint8_t version = static_cast<uint8_t>((word0 >> 13U) & 0x07U);
    if (version != 0U) {
        return std::nullopt;
    }

    // data_length = payload_size + 2 - 1 => total data field = data_length + 1 >= 2
    if (data_length < 1U) {
        return std::nullopt;
    }

    const std::size_t expected_total = kPrimaryHeaderBytes + static_cast<std::size_t>(data_length) + 1U;
    if (length != expected_total) {
        return std::nullopt;
    }

    const std::size_t crc_offset = length - kCrcBytes;
    const uint16_t expected_crc = crc16_ccitt(data, crc_offset);
    const uint16_t received_crc =
        static_cast<uint16_t>((static_cast<uint16_t>(data[crc_offset]) << 8U) | data[crc_offset + 1U]);

    if (expected_crc != received_crc) {
        return std::nullopt;
    }

    SpacePacket pkt{};
    pkt.header.version = version;
    pkt.header.type = ((word0 >> 12U) & 0x01U) != 0U ? PacketType::Telecommand : PacketType::Telemetry;
    pkt.header.secondary_header_flag = ((word0 >> 11U) & 0x01U) != 0U;
    pkt.header.apid = static_cast<uint16_t>(word0 & kMaxApid);
    pkt.header.sequence_flags = static_cast<SequenceFlags>((word1 >> 14U) & 0x03U);
    pkt.header.sequence_count = static_cast<uint16_t>(word1 & kMaxSeqCount);
    pkt.header.data_length = data_length;

    const std::size_t payload_len = crc_offset - kPrimaryHeaderBytes;
    pkt.payload.assign(data + kPrimaryHeaderBytes, data + kPrimaryHeaderBytes + payload_len);
    return pkt;
}

std::optional<SpacePacket> decode(const std::vector<uint8_t>& frame) {
    return decode(frame.data(), frame.size());
}

}  // namespace aethersat::ccsds
