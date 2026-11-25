#include "aethersat/bus/mil_std_1553.hpp"

namespace aethersat::bus {

namespace {

Word1553 pack_20bit(uint8_t sync, uint16_t payload) noexcept {
    const uint32_t parity = MilStd1553Bus::odd_parity(payload) ? 1U : 0U;
    return ((static_cast<uint32_t>(sync & 0x07U) << 17U) |
            (static_cast<uint32_t>(payload) << 1U) |
            parity);
}

bool validate_20bit(Word1553 word, uint8_t expected_sync, uint16_t& out_payload) noexcept {
    if ((word >> 20U) != 0U) {
        return false;
    }
    const uint8_t sync = static_cast<uint8_t>((word >> 17U) & 0x07U);
    if (sync != expected_sync) {
        return false;
    }
    const uint16_t payload = static_cast<uint16_t>((word >> 1U) & 0xFFFFU);
    const bool parity_bit = (word & 0x01U) != 0U;
    if (parity_bit != MilStd1553Bus::odd_parity(payload)) {
        return false;
    }
    out_payload = payload;
    return true;
}

}  // namespace

bool MilStd1553Bus::odd_parity(uint16_t payload) noexcept {
    uint16_t v = payload;
    unsigned int ones = 0;
    while (v != 0U) {
        ones += (v & 1U);
        v >>= 1U;
    }
    // Odd parity bit makes total number of 1-bits (payload + parity) odd
    return (ones % 2U) == 0U;
}

uint16_t CommandWord::to_u16() const noexcept {
    return static_cast<uint16_t>(
        ((static_cast<uint16_t>(rt_address & 0x1FU)) << 11U) |
        ((tr_transmit ? 1U : 0U) << 10U) |
        ((static_cast<uint16_t>(subaddress & 0x1FU)) << 5U) |
        (static_cast<uint16_t>(word_count & 0x1FU)));
}

Word1553 CommandWord::encode_20bit() const noexcept {
    return pack_20bit(kSyncCmdStatus, to_u16());
}

std::optional<CommandWord> CommandWord::decode_20bit(Word1553 word) noexcept {
    uint16_t payload = 0;
    if (!validate_20bit(word, kSyncCmdStatus, payload)) {
        return std::nullopt;
    }
    CommandWord cmd{};
    cmd.rt_address = static_cast<uint8_t>((payload >> 11U) & 0x1FU);
    cmd.tr_transmit = ((payload >> 10U) & 0x01U) != 0U;
    cmd.subaddress = static_cast<uint8_t>((payload >> 5U) & 0x1FU);
    cmd.word_count = static_cast<uint8_t>(payload & 0x1FU);
    return cmd;
}

uint16_t StatusWord::to_u16() const noexcept {
    return static_cast<uint16_t>(
        ((static_cast<uint16_t>(rt_address & 0x1FU)) << 11U) |
        ((message_error ? 1U : 0U) << 10U) |
        ((instrumentation ? 1U : 0U) << 9U) |
        ((service_request ? 1U : 0U) << 8U) |
        ((broadcast_cmd ? 1U : 0U) << 4U) |
        ((busy ? 1U : 0U) << 3U) |
        ((subsystem_flag ? 1U : 0U) << 2U) |
        ((dynamic_bus_ctrl ? 1U : 0U) << 1U) |
        (terminal_flag ? 1U : 0U));
}

Word1553 StatusWord::encode_20bit() const noexcept {
    return pack_20bit(kSyncCmdStatus, to_u16());
}

std::optional<StatusWord> StatusWord::decode_20bit(Word1553 word) noexcept {
    uint16_t payload = 0;
    if (!validate_20bit(word, kSyncCmdStatus, payload)) {
        return std::nullopt;
    }
    StatusWord st{};
    st.rt_address = static_cast<uint8_t>((payload >> 11U) & 0x1FU);
    st.message_error = ((payload >> 10U) & 0x01U) != 0U;
    st.instrumentation = ((payload >> 9U) & 0x01U) != 0U;
    st.service_request = ((payload >> 8U) & 0x01U) != 0U;
    st.broadcast_cmd = ((payload >> 4U) & 0x01U) != 0U;
    st.busy = ((payload >> 3U) & 0x01U) != 0U;
    st.subsystem_flag = ((payload >> 2U) & 0x01U) != 0U;
    st.dynamic_bus_ctrl = ((payload >> 1U) & 0x01U) != 0U;
    st.terminal_flag = (payload & 0x01U) != 0U;
    return st;
}

Word1553 DataWord::encode_20bit() const noexcept {
    return pack_20bit(kSyncData, value);
}

std::optional<DataWord> DataWord::decode_20bit(Word1553 word) noexcept {
    uint16_t payload = 0;
    if (!validate_20bit(word, kSyncData, payload)) {
        return std::nullopt;
    }
    return DataWord{payload};
}

void MilStd1553Bus::reset() {
    stats_ = {};
    timeout_fault_ = false;
}

void MilStd1553Bus::register_rt(uint8_t rt_address, RtHandler handler) {
    rts_[static_cast<uint8_t>(rt_address & 0x1FU)] = std::move(handler);
}

void MilStd1553Bus::unregister_rt(uint8_t rt_address) {
    rts_.erase(static_cast<uint8_t>(rt_address & 0x1FU));
}

TransactionResult MilStd1553Bus::execute_transaction(
    const CommandWord& cmd,
    const std::vector<uint16_t>& bc_to_rt_data) {
    TransactionResult result{};
    result.command_word_20b = cmd.encode_20bit();
    ++stats_.tx_frames;

    const auto decoded_cmd = CommandWord::decode_20bit(result.command_word_20b);
    if (!decoded_cmd.has_value()) {
        ++stats_.error_count;
        result.status.rt_address = cmd.rt_address;
        result.status.message_error = true;
        result.status_word_20b = result.status.encode_20bit();
        return result;
    }

    // Injected BUS_1553_TIMEOUT fault
    if (timeout_fault_) {
        ++stats_.error_count;
        result.timed_out = true;
        result.success = false;
        result.status.rt_address = cmd.rt_address;
        result.status.message_error = true;
        result.status.subsystem_flag = true;
        result.status_word_20b = result.status.encode_20bit();
        return result;
    }

    // Check if target RT exists on bus
    const auto it = rts_.find(decoded_cmd->rt_address);
    if (it == rts_.end()) {
        ++stats_.error_count;
        result.timed_out = true;
        result.success = false;
        result.status.rt_address = decoded_cmd->rt_address;
        result.status.message_error = true;
        result.status_word_20b = result.status.encode_20bit();
        return result;
    }

    const std::size_t expected_words = (decoded_cmd->word_count == 0U) ? 32U : decoded_cmd->word_count;

    // Encode & decode BC->RT data words through 20-bit wire format
    std::vector<uint16_t> wire_rx_words;
    if (!decoded_cmd->tr_transmit) {
        if (bc_to_rt_data.size() != expected_words) {
            ++stats_.error_count;
            result.status.rt_address = decoded_cmd->rt_address;
            result.status.message_error = true;
            result.status_word_20b = result.status.encode_20bit();
            return result;
        }
        wire_rx_words.reserve(bc_to_rt_data.size());
        for (uint16_t w : bc_to_rt_data) {
            const auto decoded_dw = DataWord::decode_20bit(DataWord{w}.encode_20bit());
            if (!decoded_dw.has_value()) {
                ++stats_.error_count;
                result.status.rt_address = decoded_cmd->rt_address;
                result.status.message_error = true;
                result.status_word_20b = result.status.encode_20bit();
                return result;
            }
            wire_rx_words.push_back(decoded_dw->value);
        }
    }

    auto [rt_status, rt_tx_words] =
        it->second(decoded_cmd->subaddress, decoded_cmd->tr_transmit, wire_rx_words);
    rt_status.rt_address = decoded_cmd->rt_address;
    result.status = rt_status;
    result.status_word_20b = rt_status.encode_20bit();

    // Validate RT->BC data words through 20-bit wire format
    if (decoded_cmd->tr_transmit) {
        if (rt_tx_words.size() > expected_words) {
            rt_tx_words.resize(expected_words);
        }
        result.data_words.reserve(rt_tx_words.size());
        for (uint16_t w : rt_tx_words) {
            const auto decoded_dw = DataWord::decode_20bit(DataWord{w}.encode_20bit());
            if (!decoded_dw.has_value()) {
                ++stats_.error_count;
                result.status.message_error = true;
                result.status_word_20b = result.status.encode_20bit();
                return result;
            }
            result.data_words.push_back(decoded_dw->value);
        }
    }

    ++stats_.rx_frames;
    result.success = !result.status.message_error;
    if (!result.success) {
        ++stats_.error_count;
    }
    return result;
}

}  // namespace aethersat::bus
