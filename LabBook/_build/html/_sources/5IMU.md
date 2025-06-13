# Attitude using a gyroscope and accelarometer
This section involves calculating yaw-pitch-roll (attitude) in the 3-2-1 sequence firstly using the gyroscope to measure the angular velocitiy $\hat{\omega}$ and then using Eulers method for integration to obtain a prediction for the attitude. The second part involves combining this prediction with a measurment form the accelarometer measuring $\hat{a}$ which can be used directly to calculate yaw-pitch-roll. Using the kalman filter prediction and measurment are combined to give a high accuracy estimate.


```{figure} image-28.jpg
:label: Sensor-Fusion-Kalman
Kalman filter with sensor fusion
```

## Eulers method
Using gyroscope (measures angular velocity, $\hat{\omega}$) and knowing the attitude at $t_0$ it is possible to determine the attitude of a craft at $t_k$. The relationship between the body angular velocity $\hat{\omega} = (\omega_1, \omega_2, \omega_3)$ and the time derivatives of the Euler angles $\hat{\alpha} = (\psi, \theta, \phi)$ in the 3-2-1 (yaw-pitch-roll) sequence is given by the kinematic differential equation (KDE) below:

```{margin}
$\phi$, $\theta$ and $\psi$ are euler angles that describe the orientation of the object in space. Whereas $\omega_1$, $\omega_2$, $\omega_3$  are rates of rotation i.e. roll rate pitch rate and yaw rate.

$\psi$: measures the rotation around the vertical (z) axis e.g. turning your head left and right in a "no" motion

$\theta$: measures the rotation around the sided-to-side (y) axis e.g. nodding your head up and down in a "yes" motion.

$\phi$: rotation around the front-to-back (x) axis. E.g tilting your head to touch your ear to your shoulder. 

The Euler angle rates $\dot{\phi}$, $\dot{\theta}$, $\dot{\psi}$ are not the same as body angular rates because the orientation of the body axes changes as the object rotates. 
```

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

Or more simply :


```{math}
:label: eq-simple
\hat{\dot{\alpha}} = \Phi(\hat{\alpha}) {\hat{\omega}}
```

where $\Phi(\hat{\alpha}) = \frac{1}{\cos{\theta}}\begin{bmatrix}
0 & \sin\phi  & \cos\phi  \\
0 & \cos\phi\cos\theta & -\sin\phi\cos\theta \\
\cos\theta & \sin\phi\sin\theta & \cos\phi  \sin\theta
\end{bmatrix}$ 


The predicted attitude at time $t_{k+1}$ is calculated using eulers method:
```{math}
:label: eq-euler
\hat{\alpha}^-_{k+1} = \hat{\alpha}_k + \dot{\hat{\alpha}}_k \Delta t
```

Subbing {eq}`eq-simple` into {eq}`eq-euler` gives:
```{math}
:label: euler-method2
\hat{\alpha}^-_{k+1} = \hat{\alpha}_{k} + \Phi(\hat{\alpha}_k) \hat{\omega}_k \Delta t
```
where $\Delta t = t_k - t_{k-1}$.

```{figure} image-25.png
:name: cal1
Calibration signal involved shaking the device in just the $\omega_1$ direction then pausing and shaking the device in the $\omega_1$ and $\omega_2$ directions before pausing again and shaking the deivce in the $\omega_3$. The observed motion in the $\omega_3$ was accidental.
```

Using the calibration data in {numref}`cal1`, knowing $x_0 = \vec{0}$ and {eq}`euler-method2`, attitude can be determined.

```{figure} image-29.png
:name: roll-pitch-yaw-drift-real
$x_k$ and $x^-_k$ plotted against time.
```

Overall $x^-_k$ resembles the shape of $x_k$ quite well. However as $k$ increases $x^-_k$ 'drift' away from $x_k$ in all 3 directions. The drift is least significant in the $\phi$ direction but was predicting oscillations in the third part of the calibration even though there were no oscillations in the $\omega_3$ direction, the drift also changed direction multiple times. The most significant drift was in the $\theta$ direction. Interestingly there weren't any oscillations in the first part of the calibration in the $\theta$ direction, likely because the estimation gets less accurate over time. There was also substantial drift in the $\psi$ directions as well as picking up oscilations in the x and y directions. 

Drift is caused by the error associated with the numerical integration accumulating over time. The unexpected oscillations are likely from the gyroscope being sensetive to noise. Furthermore the oscillations could be coupled meaning small oscillations in one direction can be amplified in another.

## Sensor Fusion and Kalman filters

A better way of modelling the system would be to use a kalman filter as even a noisy direct measurment of $\hat\alpha$ would correct for drift in the prediction. Try determining $A$ from {eq}`euler-method2` but this won't work since it can't be put into the form in $\hat{x}_{k+1} = A\hat{x}_k.$

```{tip}
Use Euler parameters
```

````{margin}
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
Where $\Psi(\hat{\omega}) = \frac{1}{2} \begin{bmatrix} 0 & -\omega_1 & -\omega_2 & -\omega_3 \\ \omega_1 & 0 & \omega_3 & -\omega_2 \\ \omega_2 & -\omega_3 & 0 & \omega_1 \\ \omega_3 & \omega_2 & -\omega_1 & 0\end{bmatrix}$ {cite}`Barreto2021, chapter=11.3`.
````
Below is the corresponding Euler parameters KDE:

```{math}
:label: EP KDE
\begin{bmatrix} \dot{\beta_0} \\ \dot{\beta_1} \\ \dot{\beta_2} \\ \dot{\beta_3} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} 0 & -\omega_1 & -\omega_2 & -\omega_3 \\ \omega_1 & 0 & \omega_3 & -\omega_2 \\ \omega_2 & -\omega_3 & 0 & \omega_1 \\ \omega_3 & \omega_2 & -\omega_1 & 0\end{bmatrix} \begin{bmatrix} \beta_0 \\ \beta_1 \\ \beta_2 \\ \beta_3 \end{bmatrix}
```

{cite}`Barreto2021, chapter=11.3` or more simply:

```{math}
:label: EP-KDE-simple
\vec{\dot{\beta}} = \Psi(\hat{\omega}) \vec{\beta}
```




This time the model will be defined using the Euler parameters
- $\hat{x}_k = \vec{\beta}$
- $\hat{x}_{k+1} = \hat{x}_k + \Delta t \vec{\dot{\beta}} = \hat{x}_k + \Delta t \Psi(\hat{\omega}) \hat{x}_k = \hat{x}_{k+1} = (I + \Delta t \Psi(\hat{\omega}))\hat{x}_k \implies A = (I + \Delta t \Psi(\hat{\omega}))$
- $\hat{x}^-_0 = \begin{bmatrix} 1 \\ 0 \\ 0 \\ 0 \end{bmatrix}$ since $\hat{\alpha} = \vec{0}$
- $Q = qI_{4 \times 4}$, $R = rI_{4 \times 4}$ and $P^-_0 = pI_{4 \times 4}$ to allow for simple tuning.


```{warning}
It is unlikely $Q$, $R$ and $P^-_0$ take forms like those above but in this case they allow for a sufficently good fit.
```


In this exampe the measurment from the gyroscope ($\hat\omega$) is being used to form the prediction. Furthermore knowing $\hat\omega$ isn't enough on its own to calculate yaw-pitch-roll so can't be used to calculate $\hat x$.  One solution to this would be to form a 'pseudo measurement' letting $z_k = \hat{x}^-_{k+1}$, however this doesn't contain corrective information so won't correct for drift. A better method would be sensor fusion, this involves combining different sensors to get a better estimate. 

IMU uses an accelarometer and a gyroscope which measures $\hat{a}$ and $\hat{\omega}$ respectivly.
Let:
- $\hat{z} = \vec{\beta}$ which will be determined from measurments of $\hat{a}$
- $H = I$ since there is a 1 to 1 correspondance between measurment and estimation.

In the body fixed frame given by:
```{margin}
Where $\vec{g} = \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$
```

```{math}
:label: accelarometer2
[\hat{a}]_B = [\vec{\dot{v}}]_B - [\vec{g}]_B
```



Where $\dot{v}$ is the translational accelatration and $\vec{g}$ is the accelaration due to gravity. **Assume the translational accelaration of the body is zero and the accelarometer is located at the center of rotation.** 
````{note}
B is the linear transformation matrix which traforms form the frame of the earth to the frame of the device.
```{math}
B = \begin{bmatrix}
\cos \psi \cos \theta & \sin \psi \cos \theta & -\sin \theta \\
\cos \psi \sin \theta \sin \phi - \cos \phi \sin \psi & \sin \psi \sin \theta \sin \phi + \cos \phi \cos \psi & \cos \theta \sin \phi \\
\cos \psi \sin \theta \cos \phi + \sin \phi \sin \psi & \sin \psi \sin \theta \cos \phi - \sin \phi \cos \psi & \cos \theta \cos \phi
\end{bmatrix} 
```
or written in terms of unit vectors describing the effects on the $x$, $y$ and $z$ components.
```{math}
B = \begin{bmatrix} \hat{n}_x & \hat{n}_y & \hat{n}_z \end{bmatrix}
```
where $\hat{n}_x$, $\hat{n}_y$ and $\hat{n}_z$ are 3 dimensional unit vectors in there respective directions.
````
The accelaration in the frame of the body can be calculated from its position relative to the earth:

```{math}
:label: eq-accelarometer2
[\vec{a}]_B = - B\vec{g}  = - g \hat{n}_z = \begin{bmatrix} \sin{\theta} \\ -\cos{\theta}\sin{\phi} \\ -\cos{\theta}\cos{\phi} \end{bmatrix}
```
In component form $a = \begin{bmatrix} a_1 \\ a_2 \\ a_3 \end{bmatrix}$ rearranging {eq}`eq-accelarometer2` gets:
```{math}
:label: accelarometer3
\theta = \arcsin(\frac{a_1}{g}) \quad \phi = \arcsin(\frac{-a_2}{g\cos{\theta}})
```
From the accelarometer data alone it is possible to directly calculate $\theta$ and $\phi$ but not $\psi$. In this case $\psi = 0$ since no oscillations were performed in the $\omega_3$ direction.

```{figure} Acc.png
:name: acc
yaw-pitch-roll against time from data direct measurment using the accelarometer.
```
{numref}`acc` is much noiser than the predicted data from the gyroscope, see {numref}`roll-pitch-yaw-drift-real`. However overall this data isn't suceptible to drift so on the whole is more accurate. 

## Implemementation
Several tests were carried out to ensure that the kalmen filter was implemeneted correctly.

```{note}
For tuning purposes $Q = qI$ and $R = rI$ although optimal Q and R are not always of this form it is clearly easier to modify one parameter compared to a parameter for each entry of the matrix when optimising by eye.
```
```{warning}
Additional noise has been added to the accelarometer readings to make the effects of the kalman filter more visible. Usually the accelarometer is more sensative to noise than the gyroscope, however this noise is small and hard to visualise in this example.
```
Below were tests carried out to ensure the kalman filter was correctly working. 
````{margin}
The output is $\vec{\beta}$ but can be converted into $\alpha$ using the following:
```{math}
\phi &= \arctan2\left(2(\beta_1 \beta_2 + \beta_0 \beta_3),\ \beta_0^2 + \beta_1^2 - \beta_2^2 - \beta_3^2\right) \\
\theta &= \arcsin\left(-2(\beta_1 \beta_3 - \beta_0 \beta_2)\right) \\
\psi &= \arctan2\left(2(\beta_2 \beta_3 + \beta_0 \beta_1),\ \beta_0^2 - \beta_1^2 - \beta_2^2 + \beta_3^2\right)
```
````
```{figure} Kalman_Filter_Tuning_test.png
:name: Test1
Testing the kalman filter with small $q$ and large $r$.
```

{numref}`Test1` is similar to the predicted data in {numref}`roll-pitch-yaw-drift-real` which would suggest the filter is working correctly as small $q$ and large $r$ mean the filter gives more weighting to predicted (gyroscope) data compared to measured (accelarometer) data.

```{figure} Kalman_Filter_Tuning_test2.png
:name: Test2
Testing the kalman filter with large $q$ and small $r$.
```
{numref}`Test2` is very noisy and is similar to the predicted data in {numref}`acc`, similarly this would suggest the filter is working correctly since large $q$ and small $r$ mean the filter gives more weighting to measured data compared to predicted data.

```{figure} Kalman_Filter_Tuning1.png
:name: Tuning
Kalman filter tuned optimally by eye.
```

```{figure} Kalman_Filter_Tuning_zoomed.png
:name: TuningZoomed
Zoomed in {numref}`Tuning`.
```
{numref}`Tuning` and {numref}`TuningZoomed` show the kalman filter has produced a very good fit. The filtered signal appears both noise, drift and delay free.


## Summary
The Kalman filter produced an excelent fit in this case as the prediction (from gyroscope) and measurment (from accelarometer) were complimentary to eachother. The gyroscope was less suceptible to noise but was suceptible to drift whereas the accelarometer was more suceptible to noise and less suceptible to drift. Sensor fusion gets the best of both worlds. 

```{admonition} Idea
Rewrite the part of the code that carries out kalman filter calculations and determines A in C++ as the programme runs really slowly.
```
```{bibliography}
```


