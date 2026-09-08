#!/usr/bin/env python3
"""Use a local SO101 leader to control the one Phone Arm relay session."""
from __future__ import annotations

import argparse
import asyncio
import glob
import json
import os
import sys
import time
from pathlib import Path
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

from shared.leader_protocol import LEADER_JOINTS, LEADER_MESSAGE_TYPE


DEFAULT_CALIBRATION_DIR = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"
)


def _serial_from_port(port: str) -> str:
    name = Path(port).name
    marker = "_Serial_"
    if marker in name:
        return name.split(marker, 1)[1].split("-if", 1)[0]
    return name


def discover_ports() -> list[str]:
    ports = sorted(glob.glob("/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00"))
    if not ports:
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def port_in_use(port: str) -> bool:
    """Best-effort check that prevents selecting the running destination arm."""
    target = os.path.realpath(port)
    for proc_fd_dir in Path("/proc").glob("[0-9]*/fd"):
        try:
            fds = list(proc_fd_dir.iterdir())
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(fd) == target:
                    return True
            except OSError:
                continue
    return False


def choose_port(requested: str | None) -> str:
    if requested:
        if not os.path.exists(requested):
            raise SystemExit(f"leader arm port does not exist: {requested}")
        if port_in_use(requested):
            raise SystemExit(
                f"leader arm port is already in use (possibly by the destination): {requested}"
            )
        return requested
    detected = discover_ports()
    busy = [port for port in detected if port_in_use(port)]
    ports = [port for port in detected if port not in busy]
    for port in busy:
        print(f"Ignoring arm already in use: {_serial_from_port(port)} ({port})")
    if not ports:
        raise SystemExit("no SO101 USB serial adapters found")
    if not sys.stdin.isatty():
        if len(ports) == 1:
            return ports[0]
        choices = "\n".join(f"  {i + 1}. {p}" for i, p in enumerate(ports))
        raise SystemExit(f"multiple arms detected; pass --port with one of:\n{choices}")
    print("Pick the arm to use as the leader:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {_serial_from_port(port)}  ({port})")
    while True:
        try:
            choice = int(input("Leader arm number: ").strip())
        except (ValueError, EOFError):
            choice = 0
        if 1 <= choice <= len(ports):
            return ports[choice - 1]
        print(f"Enter a number from 1 to {len(ports)}.")


def calibration_id_for(port: str, requested: str | None) -> str:
    return requested or f"soarm_{_serial_from_port(port)}"


def redact_url(url: str) -> str:
    parsed = urlsplit(url)
    query = [(k, "...") if k == "token" else (k, v) for k, v in parse_qsl(parsed.query)]
    return urlunsplit((parsed.scheme, parsed.netloc, parsed.path, urlencode(query), parsed.fragment))


def connect_leader(port: str, calibration_id: str, calibration_dir: Path):
    try:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        from lerobot.motors.feetech import OperatingMode
    except ImportError as exc:
        raise SystemExit(
            "lerobot is not installed in this Python environment; run with the project's lerobot venv"
        ) from exc

    calibration_file = calibration_dir / f"{calibration_id}.json"
    if not calibration_file.is_file():
        raise SystemExit(
            f"no calibration file for this leader: {calibration_file}\n"
            "Pass --calibration-id if the adapter was replaced but the arm retained an older identity."
        )
    leader = SO101Leader(
        SO101LeaderConfig(
            port=port,
            id=calibration_id,
            calibration_dir=calibration_dir,
            use_degrees=True,
            num_read_retries=5,
        )
    )
    # Connect without the upstream all-at-once handshake, then make the arm
    # limp before doing any other configuration. This is a hand-driven source;
    # it must never unexpectedly hold an old motor goal.
    leader.bus.connect(handshake=False)
    try:
        leader.bus.disable_torque(num_retry=3)
        for joint in LEADER_JOINTS:
            leader.bus.ping(joint, num_retry=3, raise_on_error=True)
        if not leader.is_calibrated:
            print(
                f"WARNING: leader servo calibration differs from {calibration_file}; "
                "using the file in software without writing calibration to the servos."
            )
            print(
                "Move gently after engaging: relative joint offsets are rebased, "
                "but gripper scaling may be inaccurate."
            )
        for joint in LEADER_JOINTS:
            leader.bus.write(
                "Operating_Mode", joint, OperatingMode.POSITION.value, num_retry=3
            )
        leader.bus.disable_torque(num_retry=3)
    except Exception:
        leader.bus.disconnect(disable_torque=False)
        raise
    return leader


async def receive_relay(wt, state: dict) -> None:
    while True:
        payload = await wt.recv()
        if payload is None:
            raise RuntimeError("relay closed the leader connection")
        try:
            msg = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if msg.get("type") == "relay_welcome":
            print(f"Relay connected: session={msg.get('session')} epoch={msg.get('epoch')}")
        if msg.get("ack_t") is not None:
            state["last_robot_ack"] = time.monotonic()


async def send_loop(args, leader) -> None:
    try:
        from shared.webtransport import connect_webtransport_datagrams
    except ModuleNotFoundError:
        sys.path.append("/usr/lib/python3/dist-packages")
        from shared.webtransport import connect_webtransport_datagrams

    period = 1.0 / args.hz
    source_id = f"leader-{_serial_from_port(args.port)}-{time.time_ns()}"
    seq = 0
    print(f"Connecting to {redact_url(args.url)}")
    async with connect_webtransport_datagrams(args.url, verify_tls=not args.insecure_tls) as wt:
        print("Connected. Keep the leader still while the destination anchors.")
        print("Press ENTER to engage; Ctrl-C stops and releases the destination.")
        input()
        relay_state = {"last_robot_ack": time.monotonic()}
        receiver = asyncio.create_task(receive_relay(wt, relay_state))
        start = time.monotonic()
        next_send = start
        try:
            while True:
                now = time.monotonic()
                if receiver.done():
                    await receiver
                if now - relay_state["last_robot_ack"] > args.link_timeout:
                    raise RuntimeError("no response from the destination robot; stopping leader control")
                if now < next_send:
                    await asyncio.sleep(min(next_send - now, 0.01))
                    continue
                action = leader.get_action()
                joints = {
                    joint: float(action[f"{joint}.pos"])
                    for joint in LEADER_JOINTS
                }
                seq += 1
                now_ms = int(time.time() * 1000.0)
                message = {
                    "type": LEADER_MESSAGE_TYPE,
                    "seq": seq,
                    "t": now_ms,
                    "page_id": source_id,
                    "source_id": source_id,
                    "joints": joints,
                    "enabled": True,
                }
                wt.send_latest(json.dumps(message, separators=(",", ":")).encode())
                if seq == 1:
                    print("Leader control engaged.")
                if (
                    now - start >= args.status_every
                    and seq % max(1, round(args.hz * args.status_every)) == 0
                ):
                    print(f"sent={seq} rate={seq / max(now - start, 1e-6):.1f}Hz")
                next_send += period
                if next_send < time.monotonic() - period:
                    next_send = time.monotonic() + period
        finally:
            # A release message makes the destination discard its relative anchor.
            for _ in range(3):
                seq += 1
                message = {
                    "type": LEADER_MESSAGE_TYPE,
                    "seq": seq,
                    "t": int(time.time() * 1000.0),
                    "page_id": source_id,
                    "source_id": source_id,
                    "joints": {joint: 0.0 for joint in LEADER_JOINTS},
                    "enabled": False,
                }
                try:
                    wt.send_latest(json.dumps(message, separators=(",", ":")).encode())
                    await asyncio.sleep(period)
                except Exception:
                    break
            receiver.cancel()
            try:
                await receiver
            except asyncio.CancelledError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True, help="destination's /wt/phone relay URL")
    parser.add_argument("--port", help="leader USB path; omit for an interactive list")
    parser.add_argument("--calibration-id", help="lerobot calibration ID; defaults from USB serial")
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_CALIBRATION_DIR)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--link-timeout", type=float, default=10.0)
    parser.add_argument("--status-every", type=float, default=5.0)
    parser.add_argument("--insecure-tls", action="store_true")
    args = parser.parse_args()
    if args.hz <= 0 or args.link_timeout <= 0:
        parser.error("--hz and --link-timeout must be positive")
    args.port = choose_port(args.port)
    calibration_id = calibration_id_for(args.port, args.calibration_id)
    print(f"Leader: {args.port} (calibration {calibration_id})")
    leader = connect_leader(args.port, calibration_id, args.calibration_dir)
    return_code = 0
    try:
        asyncio.run(send_loop(args, leader))
    except KeyboardInterrupt:
        print("\nStopping leader control.")
    except Exception as exc:
        print(f"Leader control stopped: {exc}", file=sys.stderr)
        return_code = 1
    finally:
        try:
            leader.bus.disable_torque(num_retry=3)
        finally:
            leader.disconnect()
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
