#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "aethersat/bus/can_bus.hpp"
#include "aethersat/bus/mil_std_1553.hpp"
#include "aethersat/bus/uart_bus.hpp"

using namespace aethersat::bus;

TEST(MilStd1553Test, CommandStatusData20BitWordFramingAndParity) {
    CommandWord cmd{};
    cmd.rt_address = kEpsRtAddress;
    cmd.tr_transmit = true;
    cmd.subaddress = 1U;
    cmd.word_count = 4U;

    const Word1553 encoded_cmd = cmd.encode_20bit();
    EXPECT_EQ((encoded_cmd >> 17U) & 0x07U, kSyncCmdStatus);
    const auto decoded_cmd = CommandWord::decode_20bit(encoded_cmd);
    ASSERT_TRUE(decoded_cmd.has_value());
    EXPECT_EQ(decoded_cmd->rt_address, kEpsRtAddress);
    EXPECT_TRUE(decoded_cmd->tr_transmit);
    EXPECT_EQ(decoded_cmd->subaddress, 1U);
    EXPECT_EQ(decoded_cmd->word_count, 4U);

    // Parity corruption must be rejected
    EXPECT_FALSE(CommandWord::decode_20bit(encoded_cmd ^ 0x01U).has_value());

    StatusWord st{};
    st.rt_address = kEpsRtAddress;
    st.service_request = true;
    st.busy = false;
    st.subsystem_flag = true;
    st.message_error = false;
    const Word1553 encoded_st = st.encode_20bit();
    const auto decoded_st = StatusWord::decode_20bit(encoded_st);
    ASSERT_TRUE(decoded_st.has_value());
    EXPECT_EQ(decoded_st->rt_address, kEpsRtAddress);
    EXPECT_TRUE(decoded_st->service_request);
    EXPECT_TRUE(decoded_st->subsystem_flag);
    EXPECT_FALSE(decoded_st->message_error);

    DataWord dw{0x5A3C};
    const Word1553 encoded_dw = dw.encode_20bit();
    EXPECT_EQ((encoded_dw >> 17U) & 0x07U, kSyncData);
    const auto decoded_dw = DataWord::decode_20bit(encoded_dw);
    ASSERT_TRUE(decoded_dw.has_value());
    EXPECT_EQ(decoded_dw->value, 0x5A3CU);
    EXPECT_FALSE(DataWord::decode_20bit(encoded_dw ^ 0x01U).has_value());
}

TEST(MilStd1553Test, RtAddressMismatchAndTimeoutFaultHandling) {
    MilStd1553Bus bus;
    bus.register_rt(
        kEpsRtAddress,
        [](uint8_t subaddr, bool tr_tx, const std::vector<uint16_t>&) {
            StatusWord st{};
            st.rt_address = kEpsRtAddress;
            if (tr_tx && subaddr == 1U) {
                return std::make_pair(st, std::vector<uint16_t>{0x1234, 0x5678});
            }
            st.message_error = true;
            return std::make_pair(st, std::vector<uint16_t>{});
        });

    // Valid transaction to RT-04
    CommandWord cmd{kEpsRtAddress, true, 1U, 2U};
    const auto ok_res = bus.execute_transaction(cmd);
    EXPECT_TRUE(ok_res.success);
    EXPECT_FALSE(ok_res.timed_out);
    ASSERT_EQ(ok_res.data_words.size(), 2U);
    EXPECT_EQ(ok_res.data_words[0], 0x1234U);
    EXPECT_EQ(ok_res.data_words[1], 0x5678U);

    // RT address mismatch -> timeout + message error
    CommandWord wrong_rt{9U, true, 1U, 2U};
    const auto miss_res = bus.execute_transaction(wrong_rt);
    EXPECT_FALSE(miss_res.success);
    EXPECT_TRUE(miss_res.timed_out);
    EXPECT_TRUE(miss_res.status.message_error);

    // BUS_1553_TIMEOUT fault injection on valid RT-04
    bus.set_timeout_fault(true);
    const auto fault_res = bus.execute_transaction(cmd);
    EXPECT_FALSE(fault_res.success);
    EXPECT_TRUE(fault_res.timed_out);
    EXPECT_TRUE(fault_res.status.message_error);
}

TEST(CanBusTest, StandardIdValidationAnd8BytePacking) {
    CanBus can;

    const std::array<double, 3> torque_cmd = {0.2500, -0.1250, 0.0500};
    const auto cmd_frame = CanBus::pack_rw_torque_cmd(torque_cmd, 0x00);
    EXPECT_EQ(cmd_frame.id, CanBus::kReactionWheelCmdId);
    EXPECT_EQ(cmd_frame.dlc, 8U);

    EXPECT_TRUE(can.transmit(cmd_frame));
    const auto rx_frame = can.receive();
    ASSERT_TRUE(rx_frame.has_value());

    const auto unpacked_cmd = CanBus::unpack_rw_torque_cmd(*rx_frame);
    ASSERT_TRUE(unpacked_cmd.has_value());
    EXPECT_NEAR((*unpacked_cmd)[0], torque_cmd[0], 1e-4);
    EXPECT_NEAR((*unpacked_cmd)[1], torque_cmd[1], 1e-4);
    EXPECT_NEAR((*unpacked_cmd)[2], torque_cmd[2], 1e-4);

    // Reject corrupted checksum
    auto bad_frame = *rx_frame;
    bad_frame.data[7] ^= 0xFFU;
    EXPECT_FALSE(CanBus::unpack_rw_torque_cmd(bad_frame).has_value());

    // Reject invalid 11-bit CAN 2.0A ID
    CanFrame invalid_id{};
    invalid_id.id = 0x800;
    invalid_id.dlc = 8;
    EXPECT_FALSE(can.transmit(invalid_id));

    // Telemetry packing (0x201)
    const std::array<double, 3> momentum = {1.234, -2.500, 0.750};
    const auto tlm_frame = CanBus::pack_rw_momentum_tlm(momentum);
    EXPECT_EQ(tlm_frame.id, CanBus::kReactionWheelTlmId);
    const auto unpacked_tlm = CanBus::unpack_rw_momentum_tlm(tlm_frame);
    ASSERT_TRUE(unpacked_tlm.has_value());
    EXPECT_NEAR((*unpacked_tlm)[0], momentum[0], 1e-3);
    EXPECT_NEAR((*unpacked_tlm)[1], momentum[1], 1e-3);
    EXPECT_NEAR((*unpacked_tlm)[2], momentum[2], 1e-3);
}

TEST(UartBusTest, SyncChecksumAndStarTrackerQuaternionRoundTrip) {
    UartBus uart(UartBus::kBaud9600);
    EXPECT_EQ(uart.baud_rate(), UartBus::kBaud9600);
    EXPECT_TRUE(uart.set_baud_rate(UartBus::kBaud115200));
    EXPECT_FALSE(uart.set_baud_rate(57600));

    const std::array<double, 4> q_in = {0.8660254, 0.0, 0.5, 0.0};
    const auto frame = uart.encode_star_tracker_quaternion(q_in);
    ASSERT_GE(frame.size(), 20U);
    EXPECT_EQ(frame[0], UartBus::kSyncByte0);
    EXPECT_EQ(frame[1], UartBus::kSyncByte1);

    const auto q_out = uart.decode_star_tracker_quaternion(frame);
    ASSERT_TRUE(q_out.has_value());
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR((*q_out)[i], q_in[i], 1e-6);
    }

    // Corrupt sync byte -> rejected
    auto bad_sync = frame;
    bad_sync[0] = 0x00;
    EXPECT_FALSE(uart.decode_star_tracker_quaternion(bad_sync).has_value());

    // Corrupt checksum -> rejected
    auto bad_csum = frame;
    bad_csum.back() ^= 0xFFU;
    EXPECT_FALSE(uart.decode_star_tracker_quaternion(bad_csum).has_value());
}
