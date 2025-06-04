# IMU

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
