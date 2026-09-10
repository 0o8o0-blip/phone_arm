"""Discover and select the physical SO101 follower arm."""
from __future__ import annotations

import sys
from pathlib import Path

from shared.usb_arm import discover_ports, port_in_use, serial_from_port


CALIBRATION_DIR = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"
)

# This deployment replaced a failed USB adapter without changing its motors.
# Keep that one hardware-to-calibration relationship while letting every normal
# installation derive its calibration identity from the selected adapter.
CALIBRATION_SERIAL_ALIASES = {
    "5B14029128": "5AE6084208",
}


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
