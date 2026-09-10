# Follower

Runtime for the physical destination arm:

- `main.py` owns the motor bus, kinematics and control loop.
- `gateway.py` receives phone or leader input from the hosted relay.
- `cameras.py` performs lightweight camera discovery during startup.
- `hardware.py` identifies the configured arm and calibration.
- `leader_mapping.py` maps leader movement onto follower joint targets.
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

No secrets, account, inbound port, TLS certificate, SSH key or tunnel are
required on a follower. Each `run.sh` creates a new anonymous robot session.
The server returns private, session-scoped runtime capabilities and `run.sh`
prints one two-hour control invitation, for example:

```sh
https://188-166-154-201.sslip.io/robot/r_example#access=...
```

The invitation secret is after `#`, so it is not included in normal HTTP access
logs. Anyone with the complete link can control that robot. Sessions are
unlisted by default; startup asks before publishing the robot in the public
directory. Multiple followers use independent control and video paths. A
temporary disconnect marks a follower offline but does not delete its session;
session state is removed only after the advertised session expiry.

Startup also lists every detected camera and a **No camera** option. The chosen
camera is published for that session. With **No camera**, arm control still
works and the browser reports that video is unavailable without retrying it.

Hosted session creation retries transient network and server failures three
times before aborting. Authentication, rate-limit and malformed-request errors
still fail immediately because retrying cannot resolve them.
