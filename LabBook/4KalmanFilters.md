# Kalman filters

**Kalman filter is essentially a low pass filter with a dynamically changing $\alpha$**. External input $z_k$ (measurement), final output $\hat{x}_k$ (estimate), final output $\hat{x}^-_k$ (prediction), error covariance $P_k$ (estimate), error covariance $P_k^-$ (prediction) system model A, H, Q and R all others are used for internal computation. 
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