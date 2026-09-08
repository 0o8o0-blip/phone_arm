# Follower

Runtime for the physical destination arm:

- `main.py` owns the motor bus, kinematics and control loop.
- `gateway.py` receives phone or leader input and serves controller metadata.
- `hardware.py` identifies the configured arm and calibration.
- `leader_mapping.py` maps leader movement onto follower joint targets.
- `recovery.py` is the manual torque-off recovery utility.
- `run.sh` starts the follower and its camera publisher.
- `robot_models/` contains runtime kinematics assets.
