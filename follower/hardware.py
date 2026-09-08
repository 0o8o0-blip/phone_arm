"""Identity of the one destination arm used by the current deployment."""
from pathlib import Path


# The original adapter failed, but its identity remains the calibration ID
# because the motors and their homing/range values did not change.
ROBOT_CALIBRATION_SERIAL = "5AE6084208"
ROBOT_ADAPTER_SERIAL = "5B14029128"
ROBOT_ID = f"soarm_{ROBOT_CALIBRATION_SERIAL}"
ROBOT_PORT = f"/dev/serial/by-id/usb-1a86_USB_Single_Serial_{ROBOT_ADAPTER_SERIAL}-if00"
ROBOT_CALIBRATION_PATH = (
    Path.home()
    / ".cache/huggingface/lerobot/calibration/robots/so_follower"
    / f"{ROBOT_ID}.json"
)
