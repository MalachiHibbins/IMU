# Kalman filters

**A kalman filter compares the predicted state, $\hat{x}^-_k$, with measurments, $\hat{z}_k$, to create an estimate of the true state, $\hat{x}_k$**. This section outlines the algorithm for the kalman filter and introduces the parameters it uses. 

```{important} 
A prediction is a forecast of the next state based on the previous state and the mathematical model. Whereas a estimate is an update of the predicted state once new measurements have been taken.
```

## Dictionary

The kalman filter deals with the linear state model. Where the state evolves linearly according to $A$:
```{math}
:label: state
x_{k+1} = Ax_k + w_k
```
and the measurement can be described as:
```{math}
:label: measurment
z_{k+1} = Hx_k + v_k 
```
- $x_k$ is the process state vector at time $t_k$ ($n\times1$ column vector)
- $z_k$ is the vector measurment at time $t_k$ ($m\times1$ column vector). 
```{margin}
When a forcing function is present the systems evolutions is effected by an additional term e.g. $x_{k+1} = Ax_k+Bu_k$. Where $B_ku_k$ models the forcing function. In the examples there is no forcing function. A forcing function is used when the output is deliberately changed e.g. changing the output from a battery by adding another cell. 
```
- $A$ is the state transition matrix which maps $x_k$ to $x_{k+1}$ in the absence of a forcing function. ($n\times n$ matrix). 
- $H$ is the state to measurement matrix giving the ideal noiseless connection between the measurement and the state vector ($m \times n$ matrix).
- $w_k$ is the linear process noise vector, noise associated with the prediction, which is noise uncorrelated over time (white sequence). E.g. the error associated with the prediction of the temperature tommorow. ($n\times1$ column vector).
- $v_k$ is the linear measurement noise vector, noise associated with the measurment. E.g. temperature measurments using a digital thermometer will be effected by random noise. ($m\times1$ column vector).
- $Q$ is the covariance matrix of $w_k$ i.e. how much the true state is expected to deviate from the predictions made by the state transition model. Large Q puts a larger weighting on $\hat{z}_k$ compared to $\hat{x}^-_k$ when predicting $\hat{x}_{k+1}$. Smaller Q puts more weighting on $\hat{x}^-_k$ compared to $\hat{z}_k$.
```{math}
:label: eq-process-noise-cov
E[w_k w_i^T] = 
\begin{cases}
    Q_k, & i = k \\
    0,   & i \neq k
\end{cases} 
``` 
{cite}`brown2012, chapter=4`
 Larger $w_k$ means $Q_k$ needs to be adjusted to be larger.
- $R$ is the covariance matrix of $v_k$. i.e. how much the measured state is effected by noise in the state transition model. $R$ tells the kalman filter how much to "trust" the measurments compared to model predictions.  Large $R$ puts more emphasis on $\hat{x}^-_k$ compared to $\hat{z}_k$ when predicting $\hat{x}_{k+1}$, small $R$ puts more emphasis on $\hat{z}_k$ compared to $\hat{x}^-_k$.
```{math}
:label: eq-measurement-noise-cov
E[v_k v_i^T] = 
\begin{cases}
    R_k, & i = k \\
    0,   & i \neq k
\end{cases} 
``` 
{cite}`brown2012, chapter=4`
. Larger $v_k$ means $R_k$ needs to be increased.
- $Q$ and $R$ are 'tuning' paramers, unlike $A$ and $H$ the filter will converge regardless there value but they will effect how 'good' the fit is.

- $e^-_k$ is the estimation error defined as: $e^-_k = x_k-\hat{x}^-_k$
- $P_k$ is then the associated error covariance matrix defined as: $P_k =E[e_ke_k^T]$ {cite}`brown2012, chapter=4`
- $K_k$ is the kalman gain which has similar effects to $\alpha$ in the low pass filter.
- $x^-_0$ is the initial state estimate. provided at the start of the estimation process.
- $P^-_0$ is the initial error estimate.
  


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

Kalman filter estimation step is like a low pass filter with dynamically changing $\alpha$.
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

```{margin}
For a more detailed derivation see {cite}`brown2012, chapter=4`.
```

```{math}
:label: eq-kalman-gain
K_k = P^-_k H^T (H P^-_k H^T + R)^{-1}
```
In this case $K_k$ is the kalman gain, which minimises $P^k$. $P^k$ is updated as:
```{margin}
For a detailed derivation of see {cite}`Barreto2021, chapter=6.4`
```
```{math}
:label: eq-error-covariance-update
P_k = P^-_k - K_k H P^-_k
```



Whereas the low pass filter passes $\hat{x}_{k-1}$ directly between time steps $t_{k-1}$ and $t_{k}$ the kalman filter predicts the next step before a measurement is carried out and compares this with the measurement to give new estimate.


```{note}
$H$ and $R$ are only used in the prediction step.
```

## Prediction step
By considering the physics of the system the predictions form a second source of knowledge for state estimation. The system dynamically changes over time and is modelled using $A$, which describes how the state is predicted to change between $t_k$ and $t_{k+1}$:

```{math}
:label: projection
\hat{x}^-_{k+1} = A_k \hat{x}_k
```

The error covariance matrix associated with this prediction $\hat{x}^-_{k+1}$ is obtained from 

```{margin}
If $\mu_x$ is transformed linearly $\mu_y = F\mu_x$ its covariance matrix, $\Sigma_x$, can be transformed using $\Sigma_y = F\Sigma_xF^T$. {cite}`Barreto2021,chapter=2`
```

```{math}
:label: projection-covariance
P_k = A_kP_kA_k^T+Q
```

$Q$ also appears here as this is how much the true state is expected to vary from the predictions. Together these two equations encode the prediction phase of the filter.  If we only had a model we wouldn't need the estimation phase i.e. we had no measurments. 


```{note}
$A$ and $Q$ are only used in the prediction step.
```

## Summary

```{figure} image.png
:name: fig-kalman-block-diagram
Block diagram of the Kalman filter process.
```


- $A$ and $H$ are essential for the kalman filter if these are incorrect the filter will not converge.
- $Q$ and $R$ are used for tuning and ensure an optimum fit.
- $x_0$ and $P_0$ are initial estimates the filter will still converge even if these are wrong. 
- $K_k$ and $P_k$ are internal parameters.
- $z_k$ are the inputs (measured state)
- $\hat{x}_k$ are the outputs (estimated state)

```{bibliography}
```