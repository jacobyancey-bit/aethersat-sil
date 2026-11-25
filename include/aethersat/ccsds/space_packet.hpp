#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace aethersat::ccsds {

enum class PacketType : uint8_t {
    Telemetry = 0,    // TM
    Telecommand = 1   // TC
};

enum class SequenceFlags : uint8_t {
    Continuation = 0x00,
    FirstSegment = 0x01,
    LastSegment  = 0x02,
    Unsegmented  = 0x03
};

struct PrimaryHeader {
    uint8_t version{0};                                          // 3 bits (000)
    PacketType type{PacketType::Telemetry};                      // 1 bit  (0=TM, 1=TC)
    bool secondary_header_flag{false};                           // 1 bit
    uint16_t apid{0};                                            // 11 bits (0..2047)
    SequenceFlags sequence_flags{SequenceFlags::Unsegmented};    // 2 bits (11 = unsegmented)
    uint16_t sequence_count{0};                                  // 14 bits (0..16383)
    uint16_t data_length{0};                                     // 16 bits (payload_size + 2 - 1)
};

struct SpacePacket {
    PrimaryHeader header{};
    std::vector<uint8_t> payload{};
};

constexpr std::size_t kPrimaryHeaderBytes = 6;
constexpr std::size_t kCrcBytes = 2;
constexpr uint16_t kMaxApid = 0x07FF;
constexpr uint16_t kMaxSeqCount = 0x3FFF;
constexpr uint16_t kCrc16CcittPoly = 0x1021;
constexpr uint16_t kCrc16CcittInit = 0xFFFF;

uint16_t crc16_ccitt(const uint8_t* data, std::size_t length) noexcept;

std::vector<uint8_t> encode(const SpacePacket& packet);

std::optional<SpacePacket> decode(const uint8_t* data, std::size_t length);
std::optional<SpacePacket> decode(const std::vector<uint8_t>& frame);

}  // namespace aethersat::ccsds
