"""End-to-end integration tests for AetherSat-SIL over ZeroMQ + CCSDS."""

from __future__ import annotations

import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterator, Tuple

import pytest
import zmq

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "ground_station"))

from ccsds_codec import (  # noqa: E402
    SpacecraftICD,
    build_tc_inject_fault,
    build_tc_set_attitude,
)


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _locate_sim_binary() -> Path:
    env_bin = os.environ.get("AETHERSAT_SIM_BIN")
    if env_bin and Path(env_bin).is_file():
        return Path(env_bin)
    candidates = [
        REPO_ROOT / "build" / "aethersat_sim",
        REPO_ROOT / "build" / "Release" / "aethersat_sim",
        REPO_ROOT / "build" / "Debug" / "aethersat_sim",
        Path("/usr/local/bin/aethersat_sim"),
    ]
    for path in candidates:
        if path.is_file():
            return path
    pytest.fail(f"Could not find compiled aethersat_sim binary in {candidates}")


@pytest.fixture()
def running_sim() -> Iterator[Tuple[str, str, SpacecraftICD]]:
    sim_bin = _locate_sim_binary()
    pub_port = _find_free_port()
    cmd_port = _find_free_port()
    pub_bind = f"tcp://127.0.0.1:{pub_port}"
    cmd_bind = f"tcp://127.0.0.1:{cmd_port}"

    proc = subprocess.Popen(
        [
            str(sim_bin),
            "--pub-endpoint",
            pub_bind,
            "--cmd-endpoint",
            cmd_bind,
            "--rate-hz",
            "50",
            "--time-scale",
            "20.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    icd = SpacecraftICD(REPO_ROOT / "icd" / "spacecraft_icd.json")

    try:
        # Allow simulator process to bind ZeroMQ sockets
        time.sleep(0.25)
        assert proc.poll() is None, f"aethersat_sim exited prematurely: {proc.stderr.read().decode()}"
        yield pub_bind, cmd_bind, icd
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


def _send_tc(cmd_endpoint: str, frame: bytes, icd: SpacecraftICD) -> dict:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 3000)
    sock.setsockopt(zmq.SNDTIMEO, 3000)
    try:
        sock.connect(cmd_endpoint)
        sock.send(frame)
        ack_raw = sock.recv()
        ack = icd.decode_telemetry(ack_raw)
        assert ack is not None, "Failed to decode TC ACK CCSDS packet"
        return ack
    finally:
        sock.close()


def test_e2e_telemetry_slew_and_1553_fault_safe_mode(
    running_sim: Tuple[str, str, SpacecraftICD],
) -> None:
    pub_ep, cmd_ep, icd = running_sim

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 2000)
    sub.connect(pub_ep)

    try:
        # 1. Verify initial telemetry reception (both APID 0x100 and 0x200)
        seen_adcs = False
        seen_eps = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (seen_adcs and seen_eps):
            raw = sub.recv()
            tm = icd.decode_telemetry(raw)
            assert tm is not None, "Received invalid CCSDS telemetry frame"
            if tm["packet_name"] == "ADCS_ORBIT_TM":
                seen_adcs = True
                assert tm["mode_name"] == "NOMINAL"
            elif tm["packet_name"] == "EPS_HEALTH_TM":
                seen_eps = True
                assert tm["bus_1553_error"] == 0
                assert 24.0 <= tm["bus_voltage_v"] <= 28.8

        assert seen_adcs, "Did not receive APID 0x100 ADCS_ORBIT_TM"
        assert seen_eps, "Did not receive APID 0x200 EPS_HEALTH_TM"

        # 2. Send CMD_SET_ATTITUDE (30-degree slew about Z axis) and verify convergence < 0.5 deg
        angle_rad = math.radians(30.0)
        target_q = [math.cos(angle_rad / 2.0), 0.0, 0.0, math.sin(angle_rad / 2.0)]
        ack_slew = _send_tc(cmd_ep, build_tc_set_attitude(target_q, seq_count=1), icd)
        assert ack_slew["status"] == 0, f"CMD_SET_ATTITUDE NACK: {ack_slew}"

        converged = False
        last_err_deg = 999.0
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            raw = sub.recv()
            tm = icd.decode_telemetry(raw)
            if tm and tm["packet_name"] == "ADCS_ORBIT_TM":
                # Check quaternion angular error directly against commanded target
                dot = abs(
                    tm["q_w"] * target_q[0]
                    + tm["q_x"] * target_q[1]
                    + tm["q_y"] * target_q[2]
                    + tm["q_z"] * target_q[3]
                )
                dot = min(1.0, max(0.0, dot))
                last_err_deg = math.degrees(2.0 * math.acos(dot))
                if last_err_deg < 0.5:
                    converged = True
                    break

        assert converged, f"Attitude failed to converge within 0.5 deg (last error: {last_err_deg:.3f} deg)"

        # 3. Inject BUS_1553_TIMEOUT fault and verify telemetry transitions to SAFE_MODE
        ack_fault = _send_tc(cmd_ep, build_tc_inject_fault("BUS_1553_TIMEOUT", seq_count=2), icd)
        assert ack_fault["status"] == 0, f"CMD_INJECT_FAULT NACK: {ack_fault}"

        safe_mode_confirmed = False
        eps_fault_confirmed = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (safe_mode_confirmed and eps_fault_confirmed):
            raw = sub.recv()
            tm = icd.decode_telemetry(raw)
            if not tm:
                continue
            if tm["packet_name"] == "ADCS_ORBIT_TM" and tm["mode_name"] == "SAFE_MODE":
                safe_mode_confirmed = True
            elif (
                tm["packet_name"] == "EPS_HEALTH_TM"
                and tm["mode_name"] == "SAFE_MODE"
                and tm["bus_1553_error"] == 1
            ):
                eps_fault_confirmed = True

        assert safe_mode_confirmed, "ADCS_ORBIT_TM did not report SAFE_MODE after BUS_1553_TIMEOUT"
        assert eps_fault_confirmed, "EPS_HEALTH_TM did not report SAFE_MODE + bus_1553_error=1"
    finally:
        sub.close()
