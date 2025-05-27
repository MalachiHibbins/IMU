# Inertial Measurment Unit (IMU) and Kalman Filters



## Sensors

IMU measures: angular rotation, force and magnetic field, measured using a: gyroscope, accelarometer and a magnetometer respectivly. Each of these measurments has serious issues: the accelarometer is easy to confuse with shaky motion but is accurate with slow steady miotion, the magnetometer will lose its pointing accuracy over time but is accurate in shaky motion. These problems are rectified using sensor fusion.

## Kalman filters

Kalman filters are a form of optimal estimation algorithm. The measurments from gyroscope, accelarometer and magnetometer are each prone to drift over time. However by combining these sensors it is possible for our estimates to converge towards the real value. The state observer $\hat{m}$ represents the estimated state vector from a set of measuremnts ($m$) which include noise. 

### 1 Reccursive Average filters

The estimated state (in this case the mean state) is evalauluated as:

\[ 
    \hat{m}_n = \frac{m_1+m_2+m_3+...+m_n}{n} \tag{1.1}
\] 


Which can be written recursivly as:
\[ 
    \hat{m}_n = (\frac{n-1}{n})\hat{m}_{n-1} + \frac{m_n}{n} \tag{1.2}
\]
which is sifnificantly easier to compute.

Let $\alpha \coloneqq \frac{n-1}{n}$ equation (1.2) can be rewritten as:

\[
    \hat{m}_n = \alpha \hat{m}_{n-1} + (1 - \alpha) m_n \tag{1.3}
\]

This is an example of an recursive (average) filter. hello

### 2 Moving Average filters

The moving average $\hat{r}_n$ is used to remove noise over a constantly varying signal. Here $n$ represents the index of the moving average and $k$ represents the window size.

\[
    \hat{m}_n = \frac{m_{m-k+1} + m_{m-k+2}+...+m_n}{k} \tag{2.1}
\]

$\hat{r}_n$ can be written reccursivly as:

\[
    \hat{m}_n = \hat{m}_{n-1} + \frac{m_n - m_{m-k}}{k} \tag{2.2}
\]

Look into the effects of $k$ on the observed real signal.

There are limitations of the moving average. All terms in $(2.1)$ have equal weighting $1/n$ however it makes more sense to give more recent terms a larger weighting.

### 3 Low pass filter

Allows low frequencies to pass through but filters out high frequencies. Noise is usually high frequency. 

\[
    \hat{m}_n = \alpha \hat{m}_{k-1} + (1 - \alpha) m_k \quad 0<\alpha<1 \tag{3.1} 
\]
Looks like expression (3.1) except here $\alpha$ is a free parameter to be chosen.  
\[
    \hat{m}_{n-1} = \alpha \hat{m}_{k-2} + (1 - \alpha) m_{k-1} \quad 0<\alpha<1 \tag{3.2}
\]
Combining these equations helps to overcome problems associated with the moving average:

\[
    \hat{m}_n = \alpha^2 \hat{m}_{n-2} + \alpha(1-\alpha) m_{n-1} + (1-\alpha)m_n \quad 0<\alpha<1 \tag{3.3}
\]
Due to the restiruction on alpha larger $n$ means greater weighting on $\hat{m}_n$ since $\alpha(1-\alpha)\leq 1-\alpha$. Previous data gets weighted exponentially less. 








