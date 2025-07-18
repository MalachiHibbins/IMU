# Intro 
This report looks at Kalman filters and how they can be used with the aim to improve undergraduate experiments. Kalman filters are a optimal estimation algorithm used to predict the true state (a description of the system) from a set of noisy measurements. The Kalman filter uses the current state along with a model, that reflects the physics of the system, to predict the next state. This is then compared with measurements to improve on this prediction. The model in the Kalman filter is dependent on how the system behaves, whereas most filters use the same algorithm with a couple of parameters that are tuned. Often the Kalman filter will use measurements from multiple types of sensors to improve accuracy, sensor fusion.

Experiment I uses a 6 axis IMU to track position and attitude. The IMU measures acceleration, in the x, y and z directions and angular velocity in the yaw, pitch and roll directions. The position and attitude estimates are inaccurate due to drift associated with integration. Using Kalman filters to fuse sensor data between the accelerometer and gyroscope as well as non inertial sensors like magnetometer and GPS would greatly improve the results of this experiment.


## Contents
```{tableofcontents}
```
 