# Kalman filters

**Kalman filter is essentially a low pass filter with a dynamically changing $\alpha$**.  States can be different usually measurements. External input $z_k$ (measurement), final output $\hat{x}_k$ (estimate), final output $\hat{x}^-_k$ (prediction), error covariance $P_k$ (estimate), error covariance $P_k^-$ (prediction) system model A, H, Q and R all others are used for internal computation. 
```{important}
A prediction is a forecast of the next state based on the previous state and the mathematical model whereas a estimate is an update of the predicted state once new measurements have been taken.
```

## Linear model

The kalman filter deals with the linear state model. Where the state:
```{math}
:label: state
x_{k+1} = Ax_k + w_k
```
and the measurement
```{math}
:label: measurment
z_{k+1} = Hx_k + v_k 
```
when no forceing function is present. 
- $x_k$ is the process state vector at time $t_k$ ($n\times1$ column vector)
- $z_k$ is the vector measurment at time $t_k$ ($m\times1$ column vector). 
- $A$ is the state transition matrix which maps $x_k$ to $x_{k+1}$ in the absence of a forcing function ($n\times n$ matrix). When a forcing function is present the systems evolutions is effected by an additional term e.g. $x_{k+1} = Ax_k+Bu_k$. Where $B_ku_k$ models the forcing function. Here it is assumed that no forceing function is present. An example of a forcing function is intentionally changing the output from a battery by adding another cell. Note this is not the same as linear process noise.
- $H$ is the state to measurement matrix giving the ideal noiseless connection between the measurement and the state vector at state vector ($m \times n$ matrix).
- $w_k$ is the linear process noise vector  which is noise uncorrelated over time (white sequence). Its values at different time steps are independent. E.g. the temperature of a fridge randomly fluctuates due to compressor inefficiencies and unpredictable random temperature shifts. ($n\times1$ column vector).
- $v_k$ is the linear measurement noise vector and is assumed to be a white sequence having zero cross corelation with $w_k$. E.g. temperature measurments using a digital thermometer will be effected by random noise($m\times1$ column vector).
- $Q$ is the covariance matrix of $w_k$ ($n\times n$ diagonal matrix) i.e. how much the true state is expected to deviate from the predictions made by the state transition model. Large Q the filter assumes the model is less reliable and vice versa.
```{math}
:label: eq-process-noise-cov
E[w_k w_i^T] = 
\begin{cases}
    Q_k, & i = k \\
    0,   & i \neq k
\end{cases}
```
- $R$ is the covariance matrix of $v_k$ ($m\times m$ diagonal matrix). $R$ tells the kalman filter how much to "trust" the measurments compared to model predictions. Large R means the measurments are trusted less so the filter relies more on its predictions.


```{math}
:label: eq-measurement-noise-cov
E[v_k v_i^T] = 
\begin{cases}
    R_k, & i = k \\
    0,   & i \neq k
\end{cases}
```

- $e^-_k$ is the estimation error defined as: $e^-_k = x_k-\hat{x}^-_k$
- $P^-_k$ is then the associated error covariance matrix defined as: $P^-_k =E[e^-_k(e^-_k)^T]$
- $x_0$ is the initial state estimate. provided at the start of the estimation process ($n \times 1$ column vector)
- $P_0$ is the initial error estimate
  
```{note}
For clarity it might be easier to write everything in dirac notation this would make it easier to distinguish what was a row/column vector and what is a matrix. It would also help to distinguish states from measurments.
```

```{figure} image.png
:name: fig-kalman-block-diagram
Block diagram of the Kalman filter process.
```

## Estimation step
The prediction $\hat{x}^-_k$ and measuremnt are combined using the blending factor $K_k$ (yet to be determined) below to determine the updated estimate $\hat{x}_k$.

```{math}
:label: eq-kalman-update
\hat{x}_k = \hat{x}^-_k + K_k(z_k - H\hat{x}^-_k)
```

````{admonition} Asside: Connection between kalman filter and low pass filter

Equation {eq}`eq-kalman-update` can be rewritten as:
```{math}
:label: eq-kalman-lowpass
\hat{x}_k = (\mathbb{I} + K_k H)\hat{x}_k + K_k z_k
```

Letting $H = \mathbb{I}$ and $\alpha= 1-K_k$ a first order low pass filter is recovered similar to {eq}`eq-lowpass-2`.

```{math}
:label: eq-kalman-alpha
\hat{x}_k = \alpha \hat{x}^-_{k} + (1 - \alpha) z_k
```

Kalman filter estimation process is like a low pass filter with dynamically changing $\alpha$ that also considers the physics of the system.
````

The minimum mean square error (MSE) is the performance criterion for the kalman filter. It is possible to form an expression for the error covariance matrix associated with the updated estimate:

```{math}
:label: eq-error-covariance
P_k = E[e_k e_k^T] = E\left[(x_k - \hat{x}_k)(x_k - \hat{x}_k)^T\right]
```
Substituting {eq}`measurment` into {eq}`eq-kalman-update` and then substituting the resulting expression into {eq}`eq-error-covariance` the expression below can be obtained by performing the indicated expectation:

```{math}
:label: P_k-minimise
P_k = (I-K_kH_k)P^-_k(I-K_kH_k)^T+K_kR_kK^T_k
```
Which is a general expression for the updated error covariance matrix. Minimising the MSE corresponds to finding the value of $K_k$ that minises the individual terms along the leading diagonal of $P_k$ as these terms represent the estimation error variances for the elements of the state vector elements being selected. This leaves: 

```{math}
:label: eq-kalman-gain
K_k = P^-_k H^T (H P^-_k H^T + R)^{-1}
```
In this case $K_k$ is the kalman gain, which minimises the mean-square estimation error. The error covariance (a measure of the inaccuracy of the estimate) is updated as:

```{math}
:label: eq-error-covariance-update
P_k = P^-_k - K_k H P^-_k
```
```{note}
$H$ and $R$ are only used in the prediction step.
```

## Prediction step
$\hat{x}_k$ is prediced from:

```{math}
:label: projection
\hat{x}^-_{k+1} = A_k \hat{x}_k
```

The error covariance matrix associated with $\hat{x}^-_{k+1}$ is obtained from 

```{math}
:label: projection-covariance
P_k = A_kP_kA_k^T+Q_k
```




```{note} 
Whereas the low pass filter passes $\hat{x}_{k-1}$ directly between time steps $t_{k-1}$ and $t_{k}$ the kalman filter predicts the next step before a measurement is carried out to produce and compares this with the measurement to give new estimate.
```

```{note}
$A$ and $Q$ are only used in the prediction step.
```



## Example 1: using a kalman filter to fit a constant signal e.g. the output from a battery
- $n = 1$ since $\hat{x}$ is a scalar, $m = 1$ since measurement $z$ is a scalar
- Assume no linear process noise ($w_k$ = 0) hence $Q = 0$ and $x_{k+1} = Ax_k$ since the voltage stays the same $x_k = x_{k+1}$ therefore $A=1$
- There measurement sensor directly observes the voltage therefore $H = 1$
- Tried different values of $R$ since there is still linear measurement noise.

```{figure} image-2.png
:name: fig-kalman-constant-signal
Kalman filter used to successfully fit a non-varying signal with a large amount of noise.
```
{numref}`fig-kalman-constant-signal` shows the rate of convergence is exponentially fast and much more smoother than the average filter making it easy to extrapolate to infinity.

```{figure} image-3.png
:name: fig-kalman-smaller-r
{numref}`fig-kalman-constant-signal` but with smaller $R$.
```

{numref}`fig-kalman-smaller-r` shows smaller R appears to make the rate of convergence faster but after 500 samples it is further from the mean than when $R=4$.

```{figure} image-4.png
:name: fig-kalman-larger-r
{numref}`fig-kalman-constant-signal` but with larger $R$.
```

{numref}`fig-kalman-larger-r` shows slower convergence than figures 4.1 and 4.2 but has the smoothest convergence of the three. {numref}`fig-kalman-constant-signal` seems to be best as it has a good tradeoff between a noise free filtered signal and fast convergence.

```{figure} image-5.png
:name: figu-kalman-wrong-h
{numref}`fig-kalman-constant-signal` with H set to $1.115$ rather than $1$
```

Varying H will lead to the kalman filter converging to the wrong value since the measurement directly measures the state only satisfied by $H = 1$.

```{figure} image-6.png
:name: fig-kalman-a-1004
{numref}`fig-kalman-constant-signal` with $A$ set to 1.004 rather than 1.
```

This causes the kalman filter to diverge since only $A=1$ describes a straight line.
```{figure} image-7.png
:name: fig-kalman-q-0817
{numref}`fig-kalman-constant-signal` except with $Q = 0.817$ rather than 1.
```
Setting Q to anything other than $0$ in this example makes the filtered signal noisy since the filter is expecting linear process noise which doesn't exist in this model because of the way it was defined..

## Example 2: estimating velocity from position
- $n = 2$ since $x_k = \begin{bmatrix}
s \\
v
\end{bmatrix}_k$ where $s$ is the position and $v$ is the velocity.
- $m = 1$ since $z_k = s_k$

- $A = 
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}$ where ($\Delta t$ is the time step between measurements) since $x_{k+1} = \begin{bmatrix}
s_k + v_k\Delta t \\ v_k 
\end{bmatrix} = Ax_k$
- $H = \begin{bmatrix}
1 &
0
\end{bmatrix}$ since $z_{k+1} = Hx_k + v_k$
- Q is given $Q = \begin{bmatrix}
Q_s & 0 \\
0 & Q_v
\end{bmatrix} = \begin{bmatrix}
1 & 0 \\
0 & 3
\end{bmatrix}$ Where $Q_s$ is the covariance in $s_k$ and $Q_v$ is the covariance in $v_k$.
- $R$ is one dimensional and is obtained by tuning.
```{important}
The true signal in the graphs below is generated using a combination of $\sin$ waves of varing amplitudes and speeds, the velocity version is the analytical deriative of this. The noisy signal is generated by adding randomly generated values to this using the guassian distribution. The kalman filter is applied to the noisy position data and outputs the filtred postion data and filtered velocity data. Which are graphed below
```
```{Note}
The summary statistics are generated between the true signal and the kalman filtered signal. $\mu^2$ refers to the random mean squared error and $\mu$ refers to the mean absolute error.
```
```{figure} image-23.png
:name: original
Kaman filter setup using parameters above to fit a noisy signal made up from sin waves with varing amplitudes and periods.
```
The fit for the position is very good with high R$^2$ value. However it could be smoother and the residules graph is not completely random suggesting there could be some delay, prehaps increasing $R$ would improve the fit. The velocity fit is poor and the filtered signal lags behind the true signal. Since only the velocity signal is lagging Q_v was increased.

```{figure} image-24.png
:name: increased-R-and-increased-Q
{numref}`original` with increased R and increased Q
```

Figure {numref}`increased-R-and-increased-Q` shows a slightly improved fit with position and a significantly improved fit with velocity. However the velocity fit is now significantly less smooth, there is a clear tradeoff between having a a smooth fit and having an accurate fit. Smoother fits more accurately represent the true shape of the data, but will often cause there to be a delay. The residuals graph for position now seems to be very random implying close to optimal fit, the residuals graph for velocity is close to being random.

```{admonition} Question
How is it possible to ensure both a smooth fit whilst minimising lag and ensuring a good fit? Prehaps a machine learning algorithm?
```

```{note}
Using a kalmamn filter for differentiation is significantly more accurate than performing numerical integration on the noisy signal. 
```





## Example 3: Dynamic orientation determination

Using gyroscope (measures angular velocity, $\hat{z}$) and knowing the initial orientation at $t_0$ it is possible to determine the altitude of a craft at $t_k$. The relationship between the body angular velocity $\hat{z} = (\omega_1, \omega_2, \omega_3)$ and the time derivatives of the Euler angles $\hat{x} = (\psi, \theta, \phi)$ in the 3-2-1 (yaw-pitch-roll) sequence is given by the kinematic differential equation (KDE) below:

```{math}
:label: H
\begin{bmatrix}
\dot{\psi} \\
\dot{\theta} \\
\dot{\phi}
\end{bmatrix} =
\frac{1}{\cos{\theta}}\begin{bmatrix}
0 & \sin\phi  & \cos\phi  \\
0 & \cos\phi\cos\theta & -\sin\phi\cos\theta \\
\cos\theta & \sin\phi\sin\theta & \cos\phi  \sin\theta
\end{bmatrix}
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
```

Or more simply:

```{math}
:label: H-simple
\hat{\dot{x}} = \Phi(\hat{x}) {\hat{z}}
```

where $\Phi(\hat{x}) = \frac{1}{\cos{\theta}}\begin{bmatrix}
0 & \sin\phi  & \cos\phi  \\
0 & \cos\phi\cos\theta & -\sin\phi\cos\theta \\
\cos\theta & \sin\phi\sin\theta & \cos\phi  \sin\theta
\end{bmatrix}$ 

```{note}
$\phi$, $\theta$ and $\psi$ are euler angles that describe the orientation of the object in space. Whereas $\omega_1$, $\omega_2$, $\omega_3$  are rates of rotation i.e. roll rate pitch rate and yaw rate.
$\psi$: measures the rotation around the vertical (z) axis e.g. turning your head left and right in a "no" motion
$\theta$: measures the rotation around the sided-to-side (y) axis e.g. nodding your head up and down in a "yes" motion.
$\phi$: rotation around the front-to-back (x) axis. E.g tilting your head to touch your ear to your shoulder. 
The Euler angle rates $\dot{\phi}$, $\dot{\theta}$, $\dot{\psi}$ are not the as the body angular rates because the orientation of the body axes changes as the object rotates. 
```

```{figure} image-25.png
:name: Calibration
Calibration signal recreated from video. Note: there seems to be accidental movement in the $\omega_3$ direction.
```

$\hat{x}_0 = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}$. By using the calibration signal seen in {numref}`Calibration` $\hat{\dot{x}}$ was calculated using {equation}`H-simple` which was then integrated using the euler method:

```{math}
:label: euler-method
x_k = x_{k-1} + \dot{x}_k \Delta t
```
$\Delta t = t_k - t_{k-1}$.

```{code-block} python
:name: Euler method
def integrate(omegas, dt, eulers_initial=np.array([0, 0, 0])):
    """
    Integrate the angular velocities to get the euler angles.
    :param omegas: A 2D numpy array of shape (n, 3) where n is the number of time steps and each row contains the angular velocities (omega_x, omega_y, omega_z).
    :param dt: The time step for integration.
    :param eulers_initial: Initial euler angles.
    """
    eulers = [eulers_initial] # A list to store all the calulated euler angles
    for omega in omegas:
        Phi_matrix = Phi(eulers[-1]) # Calculate the transformation matrix T for the current euler angles
        euler_angles_dot = Phi_matrix @ omega # Calculate the time derivative of the euler angles
        new_euler_angles = eulers[-1] + euler_angles_dot * dt 
        eulers.append(new_euler_angles)
    return np.array(eulers[:-1]) # Exclude the last element to match the length of omegas
```

```{figure} image-26.png
:name: roll-pitch-yaw-drift
The roll pitch and yaw calculated from the calibration in {numref}`Calibration` then using eulers integration method.
```

The value for roll pitch and yaw should return to zero since a full number of oscillations are completed, however in {numref}`roll-pitch-yaw-drift` this doesn't happen. The numerical integration introduces an error causing ${x}$ to "drift" away from $\hat{x}$. The drift in {numref}`roll-pitch-yaw-drift` is significant since it is larger in magnitude than the oscilation. This makes the data look like "random walks", which move further and further away from the true value as time goes on.

A better way of modelling the system would be to use a kalman filter. First determine $A$. Subbing {eq}`H-simple` into {eq}`euler-method` gets:

```{math}
:label: no-A
x_{k+1} = \vec{x}_{k} + \Delta t \Phi(x_k) \hat{z_{k}} 
```

However this won't work since it can't be put into the form in {eq}`state`.

````{tip}

Use Euler parameters

```{math}
\beta_0 = \sin\frac{\phi}{2}\sin\frac{\theta}{2}\sin\frac{\psi}{2} + \cos\frac{\phi}{2}\cos\frac{\theta}{2}\cos\frac{\psi}{2}
```
```{math}
\beta_1 = \sin\frac{\phi}{2}\cos\frac{\theta}{2}\cos\frac{\psi}{2} + \cos\frac{\phi}{2}\sin\frac{\theta}{2}\sin\frac{\psi}{2}
```
```{math}
\beta_2 = \cos\frac{\phi}{2}\sin\frac{\theta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\cos\frac{\theta}{2}\sin\frac{\psi}{2}
```
```{math}
\beta_3 = \cos\frac{\phi}{2}\cos\frac{\theta}{2}\sin\frac{\psi}{2} + \sin\frac{\phi}{2}\sin\frac{\theta}{2}\cos\frac{\psi}{2}
```

Below is the Euler parameters KDE:

```{math}
:label: EP KDE
\begin{bmatrix} \dot{\beta_0} \\ \dot{\beta_1} \\ \dot{\beta_2} \\ \dot{\beta_3} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} 0 & -\omega_1 & -\omega_2 & -\omega_3 \\ \omega_1 & 0 & \omega_3 & -\omega_2 \\ \omega_2 & -\omega_3 & 0 & \omega_1 \\ \omega_3 & \omega_2 & -\omega_1 & 0\end{bmatrix} \begin{bmatrix} \beta_0 \\ \beta_1 \\ \beta_2 \\ \beta_3 \end{bmatrix}
```

or more simply:

```{math}
:label: EP-KDE-simple
\vec{\dot{\beta}} = B(\vec{\omega}) \vec{\beta}
```

It descrubes how the Euler parameters change over time in response to the angular velocity vector $\hat{z}$. 
````

This time the model will be defined using the Euler parameters
- $x_k = \vec{\beta}$ so $n=4$
- $x_{k+1} = x_k + \Delta t \vec{\dot{\beta}} = x_k + \Delta t B x_k$ which is the equivalent of {eq}`no-A`. $x_{k+1} = (I + \Delta t B)x_k$ which is written in the same form as {eq}`state`
-  hence $A = (I + \Delta t B)$
- $H = I$
- $Q$, $R$ and $P^-_0$ can be determined by trial and error
- $\vec{\alpha}_0 = \vec{0}$ as discussed earlier so $x^-_0 = \begin{bmatrix} 1 \\ 0 \\ 0 \\ 0 \end{bmatrix}$

```{admonition} Research Idea
Look into a better way to determine $Q$ $P^-_0$. $R$ will normally be given by a sensor.
```

In this exampe the measurment from the gyroscope ($\hat\omega$) is being used to form the prediction. Furthermore knowing $\hat\omega$ isn't enough on its own to calculate yaw-pitch-roll so can't be used to calculate $\hat x$.  One solution to this would be to form a 'pseudo measurement' letting $z_k = \hat{x}^-_{k+1}$, however this doesn't contain corrective information so won't correct for drift. A better method would be sensor fusion, this involves combining different sensors to get a better estimate. 

IMU uses an accelarometer and a gyroscope which measures $\hat{a}$ in the body fixed frame given by:
```{math}
:label: accelarometer2
[\vec{a}]_B = [N\vec{\dot{v}}]_B - [\vec{g}]_B
```
Where $N\dot{v}$ is the inertial accelatration and $\vec{g}$ is the accelaration due to gravity. Assume the inertial accelaration of the body is zero and the accelarometer is located at the center of mass. This can also be written in the frame of the earth using $B = \begin{bmatrix} \hat{n}_x & \hat{n}_y & \hat{n}_z \end{bmatrix}$, where $n_x$, $n_y$ and $n_z$ are 3 dimensional unit vectors in there respective directions. Since gravity only acts in the $z$ direction:

```{math}
:label: accelarometer2
\vec{a}_B = - g \hat{n}_z = \begin{bmatrix} \sin{\theta} \\ -\cos{\theta}\sin{\phi} \\ -\cos{\theta}\cos{\phi} \end{bmatrix}
```
```{math}
:label: accelarometer3
\implies \theta = \arcsin(\frac{a_1}{g}) \quad \phi = \arcsin(\frac{-a_2}{g\cos{\theta}})
```

```{figure} image-27.png
:label: Classic-Kalman
Diagram of the inputs of the kalman filter without sensor fusion e.g. example two using velocity to calculate acceleration.
```

```{figure} image-28.png
:label: Sensor-Fusion-Kalman
Diagram of the inputs of the kalman filter with sensor fustion.
```

{numref}`Sensor-Fusion-Kalman` shows $\hat{x}_k$ and $\hat{\omega}_k$ being used to form the prediction as seen below:

```{math}
x^-_{k+1} = Ax_k =  (I + \Delta t B(\omega_k))x_k
```

The measurment comes from accelarometer readings. The prediction is 'corrected' using the data from the accelarometer.

