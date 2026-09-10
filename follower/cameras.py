"""Lightweight camera registry and discovery for follower startup."""
from __future__ import annotations

from pathlib import Path


# Pinned by physical USB path: these cameras report the same fake serial, so
# their by-id links collide when multiple devices are connected.
CAMERA_REGISTRY: tuple[tuple[str, str, str], ...] = (
    ("realflex", "RealFlex", "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1.4:1.0-video-index0"),
    ("cam2", "Cam 2", "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1.2:1.0-video-index0"),
    ("cam3", "Cam 3", "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1.4:1.0-video-index0"),
    ("c920", "C920", "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0"),
)


def present_cameras() -> list[tuple[str, str, str]]:
    cameras = [camera for camera in CAMERA_REGISTRY if Path(camera[2]).exists()]
    known_targets = {str(Path(path).resolve()) for _key, _label, path in cameras}
    by_path = Path("/dev/v4l/by-path")
    if by_path.is_dir():
        for device in sorted(by_path.glob("*-video-index0")):
            target = str(device.resolve())
            if target in known_targets:
                continue
            cameras.append((device.stem, device.name, str(device)))
            known_targets.add(target)
    return cameras
