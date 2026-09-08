# One-to-one leader-arm control

Start the destination normally:

```bash
./follower/run.sh
```

Open its authenticated web page and choose **Control with leader arm**. Copy
the displayed command to the computer that has the physical leader attached.
The command starts `controllers/leader_arm/controller.py`, ignores serial ports already in
use, and prompts you to select from the available arms.

Keep both arms still while connecting. Press Enter in the source terminal to
anchor the leader to the follower's current pose and engage control. Movement
is relative from that pose, including the gripper, so calibration differences
do not make the follower jump half-closed on engagement. Press Ctrl-C to
release control.

The saved calibration file is used to convert leader positions in software.
It is never copied into the leader servos. If the file and servo values differ,
all joints are still rebased when control engages; test the first engagement
gently.

This first version deliberately uses the relay's existing single phone/source
slot. A leader connection replaces phone control for the session, and starting
phone control replaces the leader connection. It does not implement fleet
discovery or many-to-many pairing.

Safety behavior:

- the source arm is torque-disabled before it can send controls;
- the destination holds when the stream is disabled or stale;
- recovery re-anchors at the destination's measured pose;
- destination calibration limits and the existing 200 degree/second command
  cap apply to leader commands;
- a missing destination response stops the source client.
