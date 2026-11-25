"""CCSDS 133.0-B-2 Space Packet binary codec and ICD-driven telemetry/telecommand helpers."""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

PRIMARY_HEADER_BYTES = 6
CRC_BYTES = 2
CRC16_POLY = 0x1021
CRC16_INIT = 0xFFFF

APID_ADCS_ORBIT_TM = 0x100
APID_EPS_HEALTH_TM = 0x200
APID_TELECOMMAND = 0x300
APID_TC_ACK = 0x301

CMD_SET_ATTITUDE = 0x01
CMD_SET_MODE = 0x02
CMD_INJECT_FAULT = 0x03

MODE_MAP = {0: "NOMINAL", 1: "SAFE_MODE"}
MODE_NAME_TO_ID = {v: k for k, v in MODE_MAP.items()}

FAULT_MAP = {0: "NONE", 1: "WHEEL_SATURATION", 2: "BUS_1553_TIMEOUT"}
FAULT_NAME_TO_ID = {v: k for k, v in FAULT_MAP.items()}

_TYPE_STRUCT = {
    "u8": (">B", 1),
    "i8": (">b", 1),
    "u16": (">H", 2),
    "i16": (">h", 2),
    "u32": (">I", 4),
    "i32": (">i", 4),
    "f32": (">f", 4),
    "f64": (">d", 8),
}


@dataclass
class SpacePacket:
    apid: int
    payload: bytes
    packet_type: int = 0  # 0=TM, 1=TC
    secondary_header_flag: int = 0
    sequence_flags: int = 0x03  # unsegmented
    sequence_count: int = 0
    version: int = 0
    data_length: int = 0


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC-16-CCITT (poly=0x1021, init=0xFFFF) over data."""
    crc = CRC16_INIT
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_space_packet(packet: SpacePacket) -> bytes:
    """Encode a SpacePacket into big-endian CCSDS 133.0-B-2 wire bytes with trailing CRC-16."""
    data_length = len(packet.payload) + CRC_BYTES - 1
    word0 = (
        ((packet.version & 0x07) << 13)
        | ((packet.packet_type & 0x01) << 12)
        | ((packet.secondary_header_flag & 0x01) << 11)
        | (packet.apid & 0x07FF)
    )
    word1 = ((packet.sequence_flags & 0x03) << 14) | (packet.sequence_count & 0x3FFF)
    header = struct.pack(">HHH", word0, word1, data_length)
    body = header + packet.payload
    crc = crc16_ccitt(body)
    return body + struct.pack(">H", crc)


def decode_space_packet(frame: bytes) -> Optional[SpacePacket]:
    """Decode and validate a big-endian CCSDS Space Packet frame. Returns None on error."""
    if len(frame) < PRIMARY_HEADER_BYTES + CRC_BYTES:
        return None

    word0, word1, data_length = struct.unpack(">HHH", frame[:PRIMARY_HEADER_BYTES])
    version = (word0 >> 13) & 0x07
    if version != 0 or data_length < 1:
        return None

    expected_len = PRIMARY_HEADER_BYTES + data_length + 1
    if len(frame) != expected_len:
        return None

    expected_crc = crc16_ccitt(frame[:-CRC_BYTES])
    (received_crc,) = struct.unpack(">H", frame[-CRC_BYTES:])
    if expected_crc != received_crc:
        return None

    return SpacePacket(
        version=version,
        packet_type=(word0 >> 12) & 0x01,
        secondary_header_flag=(word0 >> 11) & 0x01,
        apid=word0 & 0x07FF,
        sequence_flags=(word1 >> 14) & 0x03,
        sequence_count=word1 & 0x3FFF,
        data_length=data_length,
        payload=frame[PRIMARY_HEADER_BYTES:-CRC_BYTES],
    )


class SpacecraftICD:
    """Loads icd/spacecraft_icd.json and decodes CCSDS telemetry packets dynamically."""

    def __init__(self, icd_path: Optional[Path] = None) -> None:
        if icd_path is None:
            icd_path = Path(__file__).resolve().parent.parent / "icd" / "spacecraft_icd.json"
        with open(icd_path, "r", encoding="utf-8") as f:
            self.raw: Dict[str, Any] = json.load(f)

        self._tm_by_apid: Dict[int, Dict[str, Any]] = {}
        for _, spec in self.raw.get("telemetry", {}).items():
            self._tm_by_apid[int(spec["apid"])] = spec

        self._mode_enum = {
            int(k): v for k, v in self.raw.get("enums", {}).get("mode", {}).items()
        }
        self._fault_enum = {
            int(k): v for k, v in self.raw.get("enums", {}).get("fault", {}).items()
        }

    def decode_telemetry(self, frame: bytes) -> Optional[Dict[str, Any]]:
        pkt = decode_space_packet(frame)
        if pkt is None:
            return None

        spec = self._tm_by_apid.get(pkt.apid)
        if spec is None:
            return None
        if len(pkt.payload) < int(spec["payload_bytes"]):
            return None

        decoded: Dict[str, Any] = {
            "apid": pkt.apid,
            "packet_name": spec["name"],
            "sequence_count": pkt.sequence_count,
        }
        for field in spec["fields"]:
            fmt, size = _TYPE_STRUCT[field["type"]]
            offset = int(field["offset"])
            (raw_val,) = struct.unpack_from(fmt, pkt.payload, offset)
            scale = float(field.get("scale", 1.0))
            val: Any = raw_val * scale if scale != 1.0 else raw_val
            decoded[field["name"]] = val

        if "mode" in decoded:
            decoded["mode_name"] = self._mode_enum.get(int(decoded["mode"]), "UNKNOWN")
        if "active_fault" in decoded:
            decoded["active_fault_name"] = self._fault_enum.get(
                int(decoded["active_fault"]), "UNKNOWN"
            )
        return decoded


def build_tc_set_attitude(
    quaternion: Sequence[float], seq_count: int = 0
) -> bytes:
    """Build CCSDS TC packet for CMD_SET_ATTITUDE [qw, qx, qy, qz]."""
    if len(quaternion) != 4:
        raise ValueError("Quaternion must have 4 elements [qw, qx, qy, qz]")
    norm = math.sqrt(sum(c * c for c in quaternion))
    if norm <= 1e-12:
        raise ValueError("Quaternion norm must be non-zero")
    qn = [c / norm for c in quaternion]
    payload = struct.pack(">Bdddd", CMD_SET_ATTITUDE, qn[0], qn[1], qn[2], qn[3])
    return encode_space_packet(
        SpacePacket(
            apid=APID_TELECOMMAND,
            packet_type=1,
            sequence_count=seq_count,
            payload=payload,
        )
    )


def build_tc_set_mode(mode: str | int, seq_count: int = 0) -> bytes:
    """Build CCSDS TC packet for CMD_SET_MODE (NOMINAL or SAFE_MODE)."""
    if isinstance(mode, str):
        mode_id = MODE_NAME_TO_ID[mode.upper()]
    else:
        mode_id = int(mode)
    payload = struct.pack(">BB", CMD_SET_MODE, mode_id)
    return encode_space_packet(
        SpacePacket(
            apid=APID_TELECOMMAND,
            packet_type=1,
            sequence_count=seq_count,
            payload=payload,
        )
    )


def build_tc_inject_fault(fault: str | int, seq_count: int = 0) -> bytes:
    """Build CCSDS TC packet for CMD_INJECT_FAULT (NONE, WHEEL_SATURATION, BUS_1553_TIMEOUT)."""
    if isinstance(fault, str):
        fault_id = FAULT_NAME_TO_ID[fault.upper()]
    else:
        fault_id = int(fault)
    payload = struct.pack(">BB", CMD_INJECT_FAULT, fault_id)
    return encode_space_packet(
        SpacePacket(
            apid=APID_TELECOMMAND,
            packet_type=1,
            sequence_count=seq_count,
            payload=payload,
        )
    )
