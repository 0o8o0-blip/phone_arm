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
one-to-one: only one follower may own the `default` control session, `robot`
video path and public port-8443 bore tunnel at a time. Stop the current follower
and tunnel before connecting a replacement follower.

The replacement machine must be provisioned with the existing TURN, control
relay, MediaMTX and bore credentials. Do not generate independent credentials
on the follower; they must match the hosted services. The gateway's local TLS
certificate is generated automatically. Every `run.sh` startup also creates a
fresh two-hour browser token and prints the complete share URL. Additional
links can still be minted manually with:

```sh
./follower/mint_token.py mint --name operator --expires 2h
```
