# Follower

Runtime for the physical destination arm:

- `main.py` owns the motor bus, kinematics and control loop.
- `gateway.py` receives phone or leader input and serves controller metadata.
- `hardware.py` identifies the configured arm and calibration.
- `leader_mapping.py` maps leader movement onto follower joint targets.
- `recovery.py` is the manual torque-off recovery utility.
- `run.sh` starts the follower and its camera publisher.
- `robot_models/` contains runtime kinematics assets.

Run the follower interactively:

```sh
./follower/run.sh
```

Startup lists every detected SO101 adapter that is not already in use and asks
which arm should be the follower. Its normal calibration ID is derived from the
selected adapter serial. The menu marks arms whose calibration file is missing;
LeRobot's calibration flow runs after selection when one is required.

The existing London relay, public page, MediaMTX video service and Singapore
edge can be reused by another follower, but the deployment is currently
one-to-one: only one follower may own the `default` control session and `robot`
video path at a time. Stop the current follower before connecting its
replacement.

The replacement machine must be provisioned with the existing TURN, control
relay and MediaMTX credentials. Do not generate independent credentials on the
follower; they must match the hosted services. No inbound port, TLS certificate,
SSH key or tunnel is required on the follower. Every `run.sh` startup registers
the follower outbound, creates a fresh two-hour browser token on the VPS, and
prints the complete share URL.

```sh
./follower/run.sh
```
