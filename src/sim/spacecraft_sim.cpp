#include "aethersat/sim/spacecraft_sim.hpp"

#include <zmq.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "aethersat/ccsds/space_packet.hpp"

namespace aethersat::sim {

namespace {

void append_u8(std::vector<uint8_t>& buf, uint8_t v) {
    buf.push_back(v);
}

void append_u16_be(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>((v >> 8U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>(v & 0xFFU));
}

void append_i16_be(std::vector<uint8_t>& buf, int16_t v) {
    append_u16_be(buf, static_cast<uint16_t>(v));
}

void append_f32_be(std::vector<uint8_t>& buf, float v) {
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    buf.push_back(static_cast<uint8_t>((bits >> 24U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>((bits >> 16U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>((bits >> 8U) & 0xFFU));
    buf.push_back(static_cast<uint8_t>(bits & 0xFFU));
}

void append_f64_be(std::vector<uint8_t>& buf, double v) {
    uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int shift = 56; shift >= 0; shift -= 8) {
        buf.push_back(static_cast<uint8_t>((bits >> static_cast<unsigned>(shift)) & 0xFFU));
    }
}

double read_f64_be(const uint8_t* ptr) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; ++i) {
        bits = (bits << 8U) | static_cast<uint64_t>(ptr[i]);
    }
    double v = 0.0;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

uint16_t clamp_u16(double v) noexcept {
    return static_cast<uint16_t>(std::llround(std::clamp(v, 0.0, 65535.0)));
}

int16_t clamp_i16(double v) noexcept {
    return static_cast<int16_t>(std::llround(std::clamp(v, -32767.0, 32767.0)));
}

}  // namespace

// ---- ZmqContext / ZmqSocket RAII wrappers ----

ZmqContext::ZmqContext() : ctx_(zmq_ctx_new()) {
    if (ctx_ == nullptr) {
        throw std::runtime_error("Failed to create ZeroMQ context");
    }
}

ZmqContext::~ZmqContext() {
    if (ctx_ != nullptr) {
        zmq_ctx_term(ctx_);
        ctx_ = nullptr;
    }
}

ZmqSocket::ZmqSocket(const ZmqContext& ctx, int type) : sock_(zmq_socket(ctx.raw(), type)) {
    if (sock_ == nullptr) {
        throw std::runtime_error("Failed to create ZeroMQ socket");
    }
    const int linger_ms = 0;
    zmq_setsockopt(sock_, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));
}

ZmqSocket::~ZmqSocket() {
    if (sock_ != nullptr) {
        zmq_close(sock_);
        sock_ = nullptr;
    }
}

void ZmqSocket::bind(const std::string& endpoint) {
    if (zmq_bind(sock_, endpoint.c_str()) != 0) {
        throw std::runtime_error("zmq_bind failed for " + endpoint + ": " + zmq_strerror(errno));
    }
}

bool ZmqSocket::send(const std::vector<uint8_t>& data, int flags) {
    const int rc = zmq_send(sock_, data.data(), data.size(), flags);
    return rc >= 0;
}

bool ZmqSocket::recv(std::vector<uint8_t>& out, int flags) {
    zmq_msg_t msg;
    zmq_msg_init(&msg);
    const int rc = zmq_msg_recv(&msg, sock_, flags);
    if (rc < 0) {
        zmq_msg_close(&msg);
        return false;
    }
    const auto* ptr = static_cast<const uint8_t*>(zmq_msg_data(&msg));
    out.assign(ptr, ptr + rc);
    zmq_msg_close(&msg);
    return true;
}

// ---- SpacecraftSim ----

SpacecraftSim::SpacecraftSim() {
    // Register EPS Power Controller on MIL-STD-1553B RT-04
    eps_1553_.register_rt(
        bus::kEpsRtAddress,
        [this](uint8_t subaddress, bool tr_transmit, const std::vector<uint16_t>& /*rx_words*/) {
            bus::StatusWord st{};
            st.rt_address = bus::kEpsRtAddress;
            std::vector<uint16_t> words;

            if (tr_transmit && subaddress == 1U) {
                // Return 4 telemetry data words: solar (0.1W), load (0.1W), soc (0.01%), voltage (0.01V)
                words.push_back(clamp_u16(eps_.solar_power_w() * 10.0));
                words.push_back(clamp_u16(eps_.bus_load_w() * 10.0));
                words.push_back(clamp_u16(eps_.battery_soc_pct() * 100.0));
                words.push_back(clamp_u16(eps_.bus_voltage_v() * 100.0));
            } else if (!tr_transmit && subaddress == 2U) {
                // Command subaddress acknowledged
            } else {
                st.message_error = true;
            }
            return std::make_pair(st, words);
        });
}

void SpacecraftSim::inject_fault(FaultType fault) noexcept {
    active_fault_ = fault;
    adcs_.set_wheel_saturation_fault(fault == FaultType::WheelSaturation);
    eps_1553_.set_timeout_fault(fault == FaultType::Bus1553Timeout);
}

void SpacecraftSim::step(double dt) {
    if (dt <= 0.0) {
        return;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubStepS)));
    const double sub_dt = dt / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
        step_once(sub_dt);
    }
}

void SpacecraftSim::step_once(double dt) {
    // 1. Propagate 2-body LEO orbit
    orbit_.step(dt);
    const bool in_eclipse = orbit_.in_eclipse();

    // 2. Star Tracker -> UART -> FSW measured quaternion
    const auto uart_frame = star_tracker_uart_.encode_star_tracker_quaternion(adcs_.quaternion());
    const auto measured_q_opt = star_tracker_uart_.decode_star_tracker_quaternion(uart_frame);
    star_tracker_valid_ = measured_q_opt.has_value();
    const auto measured_q = measured_q_opt.value_or(adcs_.quaternion());

    // 3. FSW PD controller -> CAN 0x101 cmd -> Reaction Wheels -> ADCS dynamics -> CAN 0x201 tlm
    const auto cmd_torque = adcs_.compute_control_torque(measured_q);
    const auto can_cmd_frame = bus::CanBus::pack_rw_torque_cmd(
        cmd_torque, static_cast<uint8_t>(mode_));
    rw_can_.transmit(can_cmd_frame);
    const auto rx_can_cmd = rw_can_.receive();
    const auto rw_torque = rx_can_cmd.has_value()
        ? bus::CanBus::unpack_rw_torque_cmd(*rx_can_cmd).value_or(cmd_torque)
        : cmd_torque;

    adcs_.step(dt, rw_torque);

    const auto can_tlm_frame = bus::CanBus::pack_rw_momentum_tlm(
        adcs_.wheel_momentum(),
        adcs_.wheel_saturation_fault() ? 0x01U : 0x00U);
    rw_can_.transmit(can_tlm_frame);
    const auto rx_can_tlm = rw_can_.receive();
    if (rx_can_tlm.has_value()) {
        last_rw_momentum_nms_ =
            bus::CanBus::unpack_rw_momentum_tlm(*rx_can_tlm).value_or(adcs_.wheel_momentum());
    }

    // 4. EPS power update & MIL-STD-1553B BC <-> RT-04 telemetry poll
    eps_.step(dt, in_eclipse, mode_ == SpacecraftMode::SafeMode);

    bus::CommandWord eps_poll{};
    eps_poll.rt_address = bus::kEpsRtAddress;
    eps_poll.tr_transmit = true;
    eps_poll.subaddress = 1U;
    eps_poll.word_count = 4U;

    const auto tx_1553 = eps_1553_.execute_transaction(eps_poll);
    last_1553_status_word_ = tx_1553.status.to_u16();
    last_1553_error_ = (!tx_1553.success || tx_1553.timed_out || tx_1553.status.message_error);

    // 5. FDIR: MIL-STD-1553B timeout/error triggers Flight Software SAFE_MODE
    if (last_1553_error_) {
        mode_ = SpacecraftMode::SafeMode;
    }

    sim_time_s_ += dt;
}

std::vector<uint8_t> SpacecraftSim::build_tc_ack(uint8_t opcode, uint8_t status) {
    ccsds::SpacePacket pkt{};
    pkt.header.version = 0;
    pkt.header.type = ccsds::PacketType::Telemetry;
    pkt.header.secondary_header_flag = false;
    pkt.header.apid = kApidTcAck;
    pkt.header.sequence_flags = ccsds::SequenceFlags::Unsegmented;
    pkt.header.sequence_count = seq_ack_;
    seq_ack_ = static_cast<uint16_t>((seq_ack_ + 1U) & ccsds::kMaxSeqCount);
    pkt.payload = {opcode, status};
    return ccsds::encode(pkt);
}

std::vector<uint8_t> SpacecraftSim::handle_telecommand(const std::vector<uint8_t>& raw_tc) {
    const auto decoded = ccsds::decode(raw_tc);
    if (!decoded.has_value()) {
        return build_tc_ack(0x00, 1U);  // NACK: invalid CCSDS/CRC
    }
    if (decoded->header.type != ccsds::PacketType::Telecommand ||
        decoded->header.apid != kApidTelecommand ||
        decoded->payload.empty()) {
        return build_tc_ack(0x00, 1U);  // NACK: wrong APID or type
    }

    const uint8_t opcode = decoded->payload[0];
    switch (opcode) {
        case kCmdSetAttitude: {
            if (decoded->payload.size() != 1U + 4U * sizeof(double)) {
                return build_tc_ack(opcode, 1U);
            }
            const physics::Quaternion target = {
                read_f64_be(decoded->payload.data() + 1U),
                read_f64_be(decoded->payload.data() + 9U),
                read_f64_be(decoded->payload.data() + 17U),
                read_f64_be(decoded->payload.data() + 25U)
            };
            adcs_.set_target_quaternion(target);
            return build_tc_ack(opcode, 0U);
        }
        case kCmdSetMode: {
            if (decoded->payload.size() != 2U) {
                return build_tc_ack(opcode, 1U);
            }
            const uint8_t m = decoded->payload[1];
            if (m > static_cast<uint8_t>(SpacecraftMode::SafeMode)) {
                return build_tc_ack(opcode, 1U);
            }
            mode_ = static_cast<SpacecraftMode>(m);
            return build_tc_ack(opcode, 0U);
        }
        case kCmdInjectFault: {
            if (decoded->payload.size() != 2U) {
                return build_tc_ack(opcode, 1U);
            }
            const uint8_t f = decoded->payload[1];
            if (f > static_cast<uint8_t>(FaultType::Bus1553Timeout)) {
                return build_tc_ack(opcode, 1U);
            }
            inject_fault(static_cast<FaultType>(f));
            return build_tc_ack(opcode, 0U);
        }
        default:
            return build_tc_ack(opcode, 1U);
    }
}

std::vector<uint8_t> SpacecraftSim::build_adcs_orbit_tm() {
    std::vector<uint8_t> payload;
    payload.reserve(92U);

    const auto& orb = orbit_.state();
    const auto& q = adcs_.quaternion();
    const auto& w = adcs_.angular_velocity();

    append_f64_be(payload, sim_time_s_);
    append_f64_be(payload, orb[0]);
    append_f64_be(payload, orb[1]);
    append_f64_be(payload, orb[2]);
    append_f64_be(payload, orb[3]);
    append_f64_be(payload, orb[4]);
    append_f64_be(payload, orb[5]);
    append_f32_be(payload, static_cast<float>(q[0]));
    append_f32_be(payload, static_cast<float>(q[1]));
    append_f32_be(payload, static_cast<float>(q[2]));
    append_f32_be(payload, static_cast<float>(q[3]));
    append_f32_be(payload, static_cast<float>(w[0]));
    append_f32_be(payload, static_cast<float>(w[1]));
    append_f32_be(payload, static_cast<float>(w[2]));
    append_f32_be(payload, static_cast<float>(adcs_.pointing_error_deg()));
    append_u8(payload, orbit_.in_eclipse() ? 1U : 0U);
    append_u8(payload, static_cast<uint8_t>(mode_));
    append_u8(payload, static_cast<uint8_t>(active_fault_));
    append_u8(payload, 0U);

    ccsds::SpacePacket pkt{};
    pkt.header.version = 0;
    pkt.header.type = ccsds::PacketType::Telemetry;
    pkt.header.secondary_header_flag = false;
    pkt.header.apid = kApidAdcsOrbitTm;
    pkt.header.sequence_flags = ccsds::SequenceFlags::Unsegmented;
    pkt.header.sequence_count = seq_adcs_;
    seq_adcs_ = static_cast<uint16_t>((seq_adcs_ + 1U) & ccsds::kMaxSeqCount);
    pkt.payload = std::move(payload);
    return ccsds::encode(pkt);
}

std::vector<uint8_t> SpacecraftSim::build_eps_health_tm() {
    std::vector<uint8_t> payload;
    payload.reserve(32U);

    append_f64_be(payload, sim_time_s_);
    append_u16_be(payload, clamp_u16(eps_.solar_power_w() * 10.0));
    append_u16_be(payload, clamp_u16(eps_.bus_load_w() * 10.0));
    append_u16_be(payload, clamp_u16(eps_.battery_soc_pct() * 100.0));
    append_u16_be(payload, clamp_u16(eps_.bus_voltage_v() * 100.0));
    append_u8(payload, orbit_.in_eclipse() ? 1U : 0U);
    append_u8(payload, static_cast<uint8_t>(mode_));
    append_u8(payload, static_cast<uint8_t>(active_fault_));
    append_u8(payload, last_1553_error_ ? 1U : 0U);
    append_u16_be(payload, last_1553_status_word_);
    append_i16_be(payload, clamp_i16(last_rw_momentum_nms_[0] * 1000.0));
    append_i16_be(payload, clamp_i16(last_rw_momentum_nms_[1] * 1000.0));
    append_i16_be(payload, clamp_i16(last_rw_momentum_nms_[2] * 1000.0));
    append_u8(payload, star_tracker_valid_ ? 1U : 0U);
    append_u8(payload, static_cast<uint8_t>(rw_can_.stats().tx_frames & 0xFFU));
    append_u16_be(payload, 0U);

    ccsds::SpacePacket pkt{};
    pkt.header.version = 0;
    pkt.header.type = ccsds::PacketType::Telemetry;
    pkt.header.secondary_header_flag = false;
    pkt.header.apid = kApidEpsHealthTm;
    pkt.header.sequence_flags = ccsds::SequenceFlags::Unsegmented;
    pkt.header.sequence_count = seq_eps_;
    seq_eps_ = static_cast<uint16_t>((seq_eps_ + 1U) & ccsds::kMaxSeqCount);
    pkt.payload = std::move(payload);
    return ccsds::encode(pkt);
}

}  // namespace aethersat::sim
