#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include "aethersat/bus/virtual_bus.hpp"

namespace aethersat::bus {

// Authentic 20-bit MIL-STD-1553B word representation:
// Bits 19..17: 3-bit Sync pattern (Cmd/Status = 0b100, Data = 0b001)
// Bits 16..1:  16-bit payload
// Bit  0:      1-bit Odd Parity over the 16 payload bits
using Word1553 = uint32_t;

constexpr uint8_t kSyncCmdStatus = 0x04U;  // 0b100
constexpr uint8_t kSyncData      = 0x01U;  // 0b001
constexpr uint8_t kEpsRtAddress  = 4U;     // RT-04

struct CommandWord {
    uint8_t rt_address{0};   // 5 bits (0..31)
    bool tr_transmit{false}; // 1 bit (1 = RT->BC transmit, 0 = BC->RT receive)
    uint8_t subaddress{0};   // 5 bits (0..31)
    uint8_t word_count{0};   // 5 bits (0..31; 0 denotes 32 words)

    uint16_t to_u16() const noexcept;
    Word1553 encode_20bit() const noexcept;
    static std::optional<CommandWord> decode_20bit(Word1553 word) noexcept;
};

struct StatusWord {
    uint8_t rt_address{0};       // 5 bits (bits 15..11)
    bool message_error{false};   // bit 10
    bool instrumentation{false}; // bit 9
    bool service_request{false}; // bit 8
    bool broadcast_cmd{false};   // bit 4
    bool busy{false};            // bit 3
    bool subsystem_flag{false};  // bit 2
    bool dynamic_bus_ctrl{false};// bit 1
    bool terminal_flag{false};   // bit 0

    uint16_t to_u16() const noexcept;
    Word1553 encode_20bit() const noexcept;
    static std::optional<StatusWord> decode_20bit(Word1553 word) noexcept;
};

struct DataWord {
    uint16_t value{0};

    Word1553 encode_20bit() const noexcept;
    static std::optional<DataWord> decode_20bit(Word1553 word) noexcept;
};

struct TransactionResult {
    bool success{false};
    bool timed_out{false};
    Word1553 command_word_20b{0};
    StatusWord status{};
    Word1553 status_word_20b{0};
    std::vector<uint16_t> data_words{};
};

class MilStd1553Bus : public VirtualBus {
public:
    // RT handler receives (subaddress, tr_transmit, rx_data_from_bc) and returns (status, tx_data_to_bc)
    using RtHandler = std::function<std::pair<StatusWord, std::vector<uint16_t>>(
        uint8_t subaddress, bool tr_transmit, const std::vector<uint16_t>& rx_words)>;

    MilStd1553Bus() = default;

    std::string name() const override { return "MIL-STD-1553B"; }
    void reset() override;
    BusStats stats() const noexcept override { return stats_; }

    void register_rt(uint8_t rt_address, RtHandler handler);
    void unregister_rt(uint8_t rt_address);

    // BUS_1553_TIMEOUT fault injection flag
    void set_timeout_fault(bool enabled) noexcept { timeout_fault_ = enabled; }
    bool timeout_fault() const noexcept { return timeout_fault_; }

    // Execute a BC <-> RT transaction
    TransactionResult execute_transaction(
        const CommandWord& cmd,
        const std::vector<uint16_t>& bc_to_rt_data = {});

    static bool odd_parity(uint16_t payload) noexcept;

private:
    std::unordered_map<uint8_t, RtHandler> rts_;
    BusStats stats_{};
    bool timeout_fault_{false};
};

}  // namespace aethersat::bus
