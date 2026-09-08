"""Discover and select the physical SO101 follower arm."""
from __future__ import annotations

import glob
import os
import subprocess
import sys
from pathlib import Path


CALIBRATION_DIR = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"
)

# This deployment replaced a failed USB adapter without changing its motors.
# Keep that one hardware-to-calibration relationship while letting every normal
# installation derive its calibration identity from the selected adapter.
CALIBRATION_SERIAL_ALIASES = {
    "5B14029128": "5AE6084208",
}


def serial_from_port(port: str) -> str:
    name = Path(port).name
    marker = "_Serial_"
    if marker in name:
        return name.split(marker, 1)[1].split("-if", 1)[0]
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", port],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError):
        return name
    for line in out.splitlines():
        if line.startswith("ID_SERIAL_SHORT="):
            return line.split("=", 1)[1]
    return name


def discover_ports() -> list[str]:
    ports = sorted(glob.glob("/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00"))
    if not ports:
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def port_in_use(port: str) -> bool:
    """Best-effort protection against selecting an arm owned by another process."""
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


def calibration_id_for_port(port: str) -> str:
    adapter_serial = serial_from_port(port)
    calibration_serial = CALIBRATION_SERIAL_ALIASES.get(adapter_serial, adapter_serial)
    return f"soarm_{calibration_serial}"


def select_follower_arm() -> tuple[str, str]:
    detected = discover_ports()
    busy = [port for port in detected if port_in_use(port)]
    ports = [port for port in detected if port not in busy]
    for port in busy:
        print(f"Ignoring arm already in use: {serial_from_port(port)} ({port})")
    if not ports:
        raise SystemExit("no free SO101 USB serial adapters found")

    if not sys.stdin.isatty():
        if len(ports) == 1:
            port = ports[0]
            return port, calibration_id_for_port(port)
        choices = "\n".join(f"  {i + 1}. {port}" for i, port in enumerate(ports))
        raise SystemExit(f"multiple follower arms detected; run interactively and choose one:\n{choices}")

    print("Pick the arm to use as the follower:")
    for i, port in enumerate(ports, 1):
        calibration_id = calibration_id_for_port(port)
        calibration_file = CALIBRATION_DIR / f"{calibration_id}.json"
        calibration_state = "calibrated" if calibration_file.is_file() else "calibration required"
        print(
            f"  {i}. {serial_from_port(port)}  ({port})  "
            f"[{calibration_id}; {calibration_state}]"
        )
    while True:
        try:
            choice = int(input("Follower arm number: ").strip())
        except (ValueError, EOFError):
            choice = 0
        if 1 <= choice <= len(ports):
            port = ports[choice - 1]
            return port, calibration_id_for_port(port)
        print(f"Enter a number from 1 to {len(ports)}.")
