# Datasets

**PitchRollCalibration** involves motion in the pitch direction followed by a short break then motion in a combination of the pitch and Roll directions followed by a short break then motion in just the roll direction. There was no motion in the yaw direction despite the sensor picking it up.

**YawRollPitchCalibration** Involves motion in each direction followed by a short pause between changing dierections. Motion in each direction occurs on its own.

**YawRollPitchFullOrientaton** involves a full rotation in each direction. This doesn't fit very well. Likely to do with the update measurment values (measured data from magnetometer and gyroscope) being renormalised to fit inside the range $[-\pi, \pi]$ for yaw and roll and $[-\pi/2, \pi/2]$. **I was unable to fix this issue.**