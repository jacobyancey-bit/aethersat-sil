#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "aethersat/ccsds/space_packet.hpp"

using namespace aethersat::ccsds;

TEST(CcsdsSpacePacketTest, BitLevelHeaderPackingAndRoundTrip) {
    SpacePacket pkt{};
    pkt.header.version = 0;
    pkt.header.type = PacketType::Telecommand;
    pkt.header.secondary_header_flag = true;
    pkt.header.apid = 0x3A5;  // 11-bit APID
    pkt.header.sequence_flags = SequenceFlags::Unsegmented;
    pkt.header.sequence_count = 0x1234;  // 14-bit sequence counter
    pkt.payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x42};

    const auto frame = encode(pkt);
    ASSERT_EQ(frame.size(), kPrimaryHeaderBytes + pkt.payload.size() + kCrcBytes);

    // Byte 0: version(000) | type(1) | sec_hdr(1) | apid[10:8](011) = 0b0001_1011 = 0x1B
    EXPECT_EQ(frame[0], 0x1BU);
    // Byte 1: apid[7:0] = 0xA5
    EXPECT_EQ(frame[1], 0xA5U);
    // Byte 2: seq_flags(11) | seq_count[13:8](01_0010) = 0b1101_0010 = 0xD2
    EXPECT_EQ(frame[2], 0xD2U);
    // Byte 3: seq_count[7:0] = 0x34
    EXPECT_EQ(frame[3], 0x34U);
    // Bytes 4..5: data_length = payload(5) + crc(2) - 1 = 6
    EXPECT_EQ(frame[4], 0x00U);
    EXPECT_EQ(frame[5], 0x06U);

    const auto decoded = decode(frame);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->header.version, 0U);
    EXPECT_EQ(decoded->header.type, PacketType::Telecommand);
    EXPECT_TRUE(decoded->header.secondary_header_flag);
    EXPECT_EQ(decoded->header.apid, 0x3A5U);
    EXPECT_EQ(decoded->header.sequence_flags, SequenceFlags::Unsegmented);
    EXPECT_EQ(decoded->header.sequence_count, 0x1234U);
    EXPECT_EQ(decoded->header.data_length, 6U);
    EXPECT_EQ(decoded->payload, pkt.payload);
}

TEST(CcsdsSpacePacketTest, RejectsSingleBitHeaderAndPayloadCorruption) {
    SpacePacket pkt{};
    pkt.header.type = PacketType::Telemetry;
    pkt.header.apid = 0x100;
    pkt.header.sequence_count = 7;
    pkt.payload = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    const auto valid_frame = encode(pkt);
    ASSERT_TRUE(decode(valid_frame).has_value());

    // Corrupt every byte position with a single-bit flip and ensure decode() rejects it
    for (std::size_t idx = 0; idx < valid_frame.size(); ++idx) {
        auto corrupted = valid_frame;
        corrupted[idx] ^= 0x01U;
        EXPECT_FALSE(decode(corrupted).has_value())
            << "Failed to reject single-bit corruption at byte " << idx;
    }
}

TEST(CcsdsSpacePacketTest, RejectsTruncatedAndOversizedFrames) {
    SpacePacket pkt{};
    pkt.header.apid = 0x200;
    pkt.payload = {0xAA, 0xBB, 0xCC};

    const auto frame = encode(pkt);

    // Truncated frame
    std::vector<uint8_t> truncated(frame.begin(), frame.end() - 1);
    EXPECT_FALSE(decode(truncated).has_value());

    // Header-only frame
    std::vector<uint8_t> header_only(frame.begin(), frame.begin() + 6);
    EXPECT_FALSE(decode(header_only).has_value());

    // Trailing garbage byte
    auto oversized = frame;
    oversized.push_back(0x00);
    EXPECT_FALSE(decode(oversized).has_value());
}
