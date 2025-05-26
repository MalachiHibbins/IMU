# Inertial Measurment Unit (IMU) and Kalman Filters

## Theory 

### Sensors

IMU measures: angular rotation, force and magnetic field, measured using a: gyroscope, accelarometer and a magnetometer respectivly. Each of these measurments has serious issues: the accelarometer is easy to confuse with shaky motion but is accurate with slow steady miotion, the magnetometer will lose its pointing accuracy over time but is accurate in shaky motion. These problems are rectified using sensor fusion.

### Kalman filters

Kalman filters are a form of optimal estimation algorithm. The measurments from gyroscope, accelarometer and magnetometer are each prone to drift over time. However by combining these sensors it is possible for our estimates to converge towards the real value. The state observer $\hat{m}$ represents the estimated state vector from a set of $m$ measurments $z$. 

The estimated state is evalauluated as:

\[ 
    \hat{m}_n = \frac{z_1+z_2+z_3+...+z_n}{n} \tag{1.1}
\] 


Which can be written recursivly as:
\[ 
    \hat{m}_n = (\frac{n-1}{n})\hat{m}_{n-1} + \frac{z_n}{n} \tag{1.2}
\]
which is sifnificantly easier to compute.

Let $\alpha \coloneqq \frac{n-1}{n}$ equation (1.2) can be rewritten as:

\[
    \hat{m}_n = \alpha \hat{m}_{k-1} + (1 - \alpha) z_k \tag{1.3}
\]

This is an example of an recursive (average) filter. hello






