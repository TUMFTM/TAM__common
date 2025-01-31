# 3D to 2D rotations

Converting from 3d accelerations to 2D accelerations is a complicated endeavor.
Therefore, this package provides helper functions for transforming into the correct 2D frame the control module
expects.
See the following image:

![](./doc/accel_overview_3d.drawio.svg)

Be sure to use raw imu accelerations (containing gravitational effects) before transforming into 2D. For planning, this corresponds to the tilde frame.

