# Kalman Filters and IMU

## Sensors

IMU measures: angular rotation, force and magnetic field, measured using a: gyroscope, accelerometer and a magnetometer respectively. Each of these measurements has serious issues: the accelerometer is easy to confuse with shaky motion but is accurate with slow steady motion, the magnetometer will lose its pointing accuracy over time but is accurate in shaky motion. These problems are rectified using sensor fusion.

## Kalman Filters

Kalman filters are a form of optimal estimation algorithm. The measurements from gyroscope, accelerometer and magnetometer are each prone to drift over time. However by combining these sensors it is possible for our estimates to converge towards the real value. The state observer $\hat{x}$ represents the estimated state vector from a set of measurements ($z$) which include noise. 

```{tableofcontents}
```
