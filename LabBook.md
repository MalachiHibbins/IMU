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

Allows low frequencies to pass through but filters out high frequencies. Noise is usually high frequency. Below is an example of a first order low pass filter.

\[
    \hat{m}_n = \alpha \hat{m}_{k-1} + (1 - \alpha) m_k \quad 0<\alpha<1 \tag{3.1} 
\]
Looks like expression (1.3) except here $\alpha$ is a free parameter to be chosen.  
\[
    \hat{m}_{n-1} = \alpha \hat{m}_{k-2} + (1 - \alpha) m_{k-1} \quad 0<\alpha<1 \tag{3.2}
\]
Combining these equations helps to overcome problems associated with the moving average:

\[
    \hat{m}_n = \alpha^2 \hat{m}_{n-2} + \alpha(1-\alpha) m_{n-1} + (1-\alpha)m_n \quad 0<\alpha<1 \tag{3.3}
\]
Due to the restiruction on alpha larger $n$ means greater weighting on $\hat{m}_n$ since $\alpha(1-\alpha)\leq 1-\alpha$. Previous data gets weighted exponentially less. 

### 4 Kalman filters

**Kalman filter is essentially a low pass filter with a dynamically changin $\alpha$**.  States can be different usually meansuments. External input $z_k$ (measurment), final output $\hat{x}_k$ (estimate), final output $\hat{x}^-_k$ (prediction), error covariance $P_k$ (estimate), error covariance $P_k^-$ (prediction) system model A, H, Q and R all others are used for internal computation. A prediction is a forcast of the next state based on the previous state and the mathematical model whereas a estimate is an update of the predicted state once new measurments have been taken.
![there](image.png)

#### Estimation step
Computation of an estimate (where $x_k$ is a $n$x1 column vector) can be written as:
\[
\hat{x}_k = \hat{x}^-_k + K_k(z_k-H\hat{x}^-_k) \tag{4.1}
\]

rewritten as
\[
\hat{x}_k = (\mathbb{I} + K_kH)\hat{x}_k + K_kz_k \tag{4.2}
\]

Letting $H = \mathbb{I}$ and $\alpha= 1-K_k$ a first order low pass filter is recovered similar to (3.1).

\[
\hat{x}_k = \alpha \hat{x}^-_{k-1} + (1 - \alpha) z_k \tag{4.3} 
\] 

**Kalman filtere estimation process is like a low pass filter with dynamically changing $\alpha$ (or equivalently gain $K_k$)** The gain gets updated in time according to $K_k$:

\[
K_k = P^-_kH^T(HP^-_kH^T+R)^{-1} \tag{4.4}
\]

The error covariance (a measure of the innaccuracy of the estimate) is updated as:
\[
P_k = P^-_k-K_kHP^-_k \tag{4.5}
\]
The true state is $x_k \sim N(\hat{x}_k, P_k)$ $\hat{x}$ is the mean and $P_k$ is the variance. *R and H are only in the estimation step*

#### Prediction step
Using the model $\hat{x}_{k+1}$ can be predicted from the state at time $t_k$, $\hat{x}^-_{k+1} = A\hat{x}_{k}$, where A is a $n\times n$ matrix. Similarly $P^-_k = AP_kA^T + Q$. Where $Q$ is the process noise or state transition noise. *A and Q are only in the prediction step*. This means (4.1) can be rewritten as:

\[
\hat{x}_k = A \hat{x}_{k-1} + K_k(z_k-HA \hat{x}_{k-1}) \tag{4.6}
\]
#### Linear model
Whereas the low pass filter passes $\hat{x}_{k-1}$ directly between timesteps $t_{k-1}$ and $t_{k}$ the kalman filter predicts the next step before a measurment is carried out to produce a new estimate.

The kalman filter deals with the linear state model. Where the state $x_{k+1} = Ax_k + w_k$ and the measurment $z_{k+1} = Hx_k + v_k$. 
- $x_k$ is the state variable ($n\times1$ column vector)
- $z_k$ is the measurment ($m\times1$ column vector). 
- $A$ is the state tranasition matrix ($n\times n$ matrix)
- $H$ is the state to measurment matrix ($m \times n$ matrix).
- $w_k$ is the linear process noise ($n\times1$ column vector)
- $v_k$ is the linear measurment noise ($m\times1$ column vector)
- $Q$ is the covariance matrix of $w_k$ ($n\times n$ diagonal matrix)
- $R$ is the covariance matrix of $v_k$ ($m\times m$ diagonal matrix)




