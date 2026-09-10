"""Shared USB serial discovery helpers for SO101 arms."""
from __future__ import annotations

import glob
import os
import subprocess
from pathlib import Path


def serial_from_port(port: str) -> str:
    """Return the adapter serial when available, otherwise the device name."""
    name = Path(port).name
    marker = "_Serial_"
    if marker in name:
        return name.split(marker, 1)[1].split("-if", 1)[0]
    try:
        output = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", port],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError):
        return name
    for line in output.splitlines():
        if line.startswith("ID_SERIAL_SHORT="):
            return line.split("=", 1)[1]
    return name


def discover_ports() -> list[str]:
    ports = sorted(glob.glob("/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00"))
    if not ports:
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def port_in_use(port: str) -> bool:
    """Best-effort check that an arm is not already owned by another process."""
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
