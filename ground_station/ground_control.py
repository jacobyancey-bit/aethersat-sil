#!/usr/bin/env python3
"""AetherSat-SIL Python Ground Control Station CLI."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import zmq

from ccsds_codec import (
    SpacecraftICD,
    build_tc_inject_fault,
    build_tc_set_attitude,
    build_tc_set_mode,
)


def send_telecommand(
    cmd_endpoint: str, tc_frame: bytes, icd: SpacecraftICD, timeout_ms: int = 3000
) -> Optional[dict]:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    try:
        sock.connect(cmd_endpoint)
        sock.send(tc_frame)
        reply = sock.recv()
        return icd.decode_telemetry(reply)
    finally:
        sock.close()


def stream_telemetry(
    pub_endpoint: str, icd: SpacecraftICD, max_packets: int = 0, timeout_ms: int = 2000
) -> None:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.connect(pub_endpoint)

    count = 0
    latest_adcs: Optional[dict] = None
    latest_eps: Optional[dict] = None

    print(f"[ground_control] Subscribed to {pub_endpoint} (Ctrl-C to exit)")
    try:
        while True:
            try:
                raw = sock.recv()
            except zmq.Again:
                continue

            tm = icd.decode_telemetry(raw)
            if tm is None:
                continue

            count += 1
            if tm["packet_name"] == "ADCS_ORBIT_TM":
                latest_adcs = tm
            elif tm["packet_name"] == "EPS_HEALTH_TM":
                latest_eps = tm

            if latest_adcs and latest_eps:
                print(
                    f"t={latest_adcs['sim_time_s']:7.2f}s | "
                    f"MODE={latest_adcs['mode_name']:9s} | "
                    f"FAULT={latest_adcs['active_fault_name']:16s} | "
                    f"q=[{latest_adcs['q_w']:+.3f},{latest_adcs['q_x']:+.3f},"
                    f"{latest_adcs['q_y']:+.3f},{latest_adcs['q_z']:+.3f}] "
                    f"err={latest_adcs['pointing_err_deg']:5.2f}deg | "
                    f"EPS={latest_eps['solar_power_w']:5.1f}W/{latest_eps['bus_load_w']:4.1f}W "
                    f"SoC={latest_eps['battery_soc_pct']:5.2f}% V={latest_eps['bus_voltage_v']:5.2f}V "
                    f"1553_ERR={latest_eps['bus_1553_error']}"
                )
            elif latest_adcs:
                print(
                    f"t={latest_adcs['sim_time_s']:7.2f}s | "
                    f"MODE={latest_adcs['mode_name']} | err={latest_adcs['pointing_err_deg']:.2f}deg"
                )

            if max_packets > 0 and count >= max_packets:
                break
    except KeyboardInterrupt:
        print("\n[ground_control] Stopped.")
    finally:
        sock.close()


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AetherSat-SIL Ground Control CLI")
    parser.add_argument(
        "--pub-endpoint",
        default="tcp://localhost:5555",
        help="ZeroMQ PUB telemetry endpoint (default: tcp://localhost:5555)",
    )
    parser.add_argument(
        "--cmd-endpoint",
        default="tcp://localhost:5556",
        help="ZeroMQ REP telecommand endpoint (default: tcp://localhost:5556)",
    )
    parser.add_argument(
        "--icd",
        type=Path,
        default=None,
        help="Path to spacecraft_icd.json (default: ../icd/spacecraft_icd.json)",
    )
    parser.add_argument(
        "--slew",
        nargs=4,
        type=float,
        metavar=("QW", "QX", "QY", "QZ"),
        help="Send CMD_SET_ATTITUDE target quaternion [qw qx qy qz]",
    )
    parser.add_argument(
        "--safe-mode",
        action="store_true",
        help="Send CMD_SET_MODE to transition spacecraft into SAFE_MODE",
    )
    parser.add_argument(
        "--nominal-mode",
        action="store_true",
        help="Send CMD_SET_MODE to transition spacecraft into NOMINAL mode",
    )
    parser.add_argument(
        "--inject-fault",
        choices=["NONE", "WHEEL_SATURATION", "BUS_1553_TIMEOUT"],
        help="Send CMD_INJECT_FAULT telecommand",
    )
    parser.add_argument(
        "--listen",
        action="store_true",
        help="Subscribe and display live decoded telemetry",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of telemetry packets to display before exiting (0 = unlimited)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    icd = SpacecraftICD(args.icd)
    did_cmd = False

    if args.slew is not None:
        ack = send_telecommand(args.cmd_endpoint, build_tc_set_attitude(args.slew), icd)
        print(f"[TC] CMD_SET_ATTITUDE -> ACK={ack}")
        did_cmd = True

    if args.safe_mode:
        ack = send_telecommand(args.cmd_endpoint, build_tc_set_mode("SAFE_MODE"), icd)
        print(f"[TC] CMD_SET_MODE(SAFE_MODE) -> ACK={ack}")
        did_cmd = True
    elif args.nominal_mode:
        ack = send_telecommand(args.cmd_endpoint, build_tc_set_mode("NOMINAL"), icd)
        print(f"[TC] CMD_SET_MODE(NOMINAL) -> ACK={ack}")
        did_cmd = True

    if args.inject_fault is not None:
        ack = send_telecommand(
            args.cmd_endpoint, build_tc_inject_fault(args.inject_fault), icd
        )
        print(f"[TC] CMD_INJECT_FAULT({args.inject_fault}) -> ACK={ack}")
        did_cmd = True

    if args.listen or not did_cmd:
        time.sleep(0.05)
        stream_telemetry(args.pub_endpoint, icd, max_packets=args.count)

    return 0


if __name__ == "__main__":
    sys.exit(main())
