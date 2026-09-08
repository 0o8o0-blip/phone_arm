# SO101 kinematics model

`follower/main.py` loads `so101_new_calib.urdf` through LeRobot's
`RobotKinematics` class. The model supplies the SO101 link and joint geometry
used by forward and inverse kinematics during follower control.

The STL files under `assets/` are referenced by the URDF and must remain beside
it for the current Placo model loader. They are runtime model assets, not a set
of printable parts maintained by this project.

Editable CAD source files and MuJoCo models are outside the scope of this
repository.
