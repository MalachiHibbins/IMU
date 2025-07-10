# 2 Kalman filters

**A Kalman filter compares the predicted state, $\hat{x}^-_k$, with measurements, $z_k$, to create an estimate of the true state, $\hat{x}_k$**. This section introduces the Kalman filter, its algorithm and why its useful. The state is a vector that contains all the variables required to describe the system at a specific time.

```{important} 
Predictions and estimates are not interchangeable when talking about Kalman filters. A prediction is a forecast of the next state based on the previous state and the mathematical model. Whereas a estimate is an update of the predicted state once new measurements have been taken.
```
```{note}
In all future sections scalars will be written as plain text $a$ and vectors will be written in bold $\boldsymbol{a}$. Here the symbols are context free so so are all in plain text.
```

## 2.1 Dictionary

The Kalman filter deals with the linear state model. Where the state evolves linearly according to $A$:
```{math}
:label: state
x_{k+1} = Ax_k + w_k
```
As long as no forcing function is present. The measurement can be described as:
```{math}
:label: measurement
z_k = Hx_k + v_k 
```
Where $w_k$ and $v_k$ are both distributed normally with mean $0$.
```{margin}
A forcing function is something external that **deliberately** changes how the system behaves. E.g. When modelling a car rolling down a hill the cars motion is determined by gravity. Applying the cars breaks would be an example of a forcing function.
```
- $x_k$ is the actual (but unknown) state vector at time $t_k$ e.g. the true position and velocity of a car. $x_k$. ($n\times 1$ column vector).
- $z_k$ is the measurement of the state from the sensor at time $t_k$, but contains noise and errors e.g. the GPS position of the car. ($m \times 1$ column vector).
- $A$ is the state transition matrix. $A$ represents the "rules of motion" of the system. It describes how $\hat{x}_k$ evolves from one time step to the next, assuming there is no external factors changing it (a forcing function). For example if you knew a cars velocity at time $t_k$ you could use $A$ to predict where it would be at time $t_{k+1}$. ($n\times n$ matrix). 
```{note}
$A$ isn't always a constant matrix, it can be dependent on time, or measurements from other sensors.
```
- $\hat{x}_k$ and $\hat{x}^-_k$ represent the best estimate and prediction of $x_k$ respectively. $\hat{x}^-_k$ is a prediction of the next state based on the physics of the system which is modeled using $A$. $\hat{x}_k$ is a updated estimate which blends the model based prediction $\hat{x}^-_k$ and the measurement $z_k$. $\hat{x}_k$ is the end output from the Kalman filter.
- $H$ is the state to measurement matrix. $H$ is the translator between the system's state and what can be measured. It explains how, if there were no noise or errors, the true state would appear in the sensor. E.g. if the model uses position to predict velocity but the sensor only measures position how $H$ only picks out position. ($m \times n$ matrix). The measurement can be estimated using $\hat{x}_k$.
- $\hat{z}^-_k$ and $\hat{z}_k$ is the what the model predicts and estimates our measurements should be respectively:
```{math}
:label: eq-h-calculate
\hat{z}_k &= H\hat{x}_k

\hat{z}^-_k &= H\hat{x}^-_k
```
{eq}`eq-h-calculate` is useful for determining $H$.
- $w_k$ is the linear process noise vector or noise associated with the prediction. $w_k$ is white sequence noise (random noise uncorrelated with time), which makes the models prediction imperfect. E.g. unexpected bumps in the road for a moving car. ($n \times 1$ column vector).
- $v_k$ is the linear measurement noise vector, noise associated that corrupts $z_k$. Even if the true state is fixed, our measurements can change due to sensor imperfections. E.g. temperature measurements using a thermometer will be slightly different each time you measure. ($m \times 1$ column vector).
- $Q$ is the covariance matrix of $w_k$ i.e. how much the true state is expected to deviate from the predictions made by the state transition model. Large $Q$ assumes the measurements are more reliable than the model and puts a larger weighting on $z_k$ compared to $\hat{x}^-_k$ when computing $\hat{x}_{k+1}$. Smaller Q puts more trust in the model $\hat{x}^-_k$ compared to $z_k$.
```{math}
:label: eq-process-noise-cov
Q_k = \mathbb{E}[w_k w_k^T]
``` 
{cite}`brown2012, chapter=4`
- $R$ is the covariance matrix of $v_k$. i.e. how much the measured state is effected by noise in the state transition model. $R$ tells the Kalman filter how much to "trust" the measurements compared to model predictions.  Large $R$ suggests the model is more reliable than the measurements and puts more emphasis on $\hat{x}^-_k$ compared to $z_k$ when computing $\hat{x}_{k+1}$, small $R$ puts more emphasis on $z_k$ compared to $\hat{x}^-_k$.
```{math}
:label: eq-measurement-noise-cov
R_k = \mathbb{E}[v_k v_k^T]
``` 
{cite}`brown2012, chapter=4`
- $e_k$ and $e^-_k$ are the error in the estimation and the error in the prediction respectively defined as $e_k = x_k-\hat{x}_k$ and $e^-_k = x_k-\hat{x}^-_k$.
- $P_k$ and $P^-_k$ are the associated error covariance matrices for $e_k$ and $e^-_k$ respectively defined by:
```{math}
:label: eq-estimation-cov
P_k = \mathbb{E}[e_k e_k^T]
```
and
```{math}
:label: eq-prediction-cov
P^-_k = \mathbb{E}[e^-_k (e^-_k)^T]
```
{cite}`brown2012, chapter=4`
- $K_k$ is the Kalman gain which has similar effects to $\alpha$ in the low pass filter.
- $x_0$ is the initial state estimate. provided at the start of the estimation process.
- $P_0$ is the initial error covariance matrix estimate.
```{important}
The Kalman filter parameters are the parameters set by the user before fitting the data they are: $A$, $Q$, $H$, $R$, $\hat{x}_0$ and $P_0$. 
```
  

## 2.2 Estimation step
The prediction $\hat{x}^-_k$ and measurement are combined using the blending factor $K_k$ (yet to be determined) below to determine the updated estimate $\hat{x}_k$.

```{math}
:label: eq-kalman-update
\hat{x}_k = \hat{x}^-_k + K_k(z_k - H\hat{x}^-_k)
```

this is known as the **update step** $K_k$ determines how much of each $z_k$ and $\hat{x}^-_k$ goes into $\hat{x}_k$ i.e. how much we comparatively trust the measurement and the prediction.

````{admonition} Asside: Connection between Kalman filter and low pass filter

Equation {eq}`eq-kalman-update` can be rewritten as:
```{math}
:label: eq-kalman-lowpass
\hat{x}_k = (\mathbb{I} + K_k H)\hat{x}^-_k + K_k z_k
```

Letting $H = \mathbb{I}$ and $\alpha= 1-K_k$ a first order low pass filter is recovered similar to {eq}`eq-lowpass-2`.

```{math}
:label: eq-kalman-alpha
\hat{x}_k = \alpha \hat{x}^-_{k} + (1 - \alpha) z_k
```

A low pass filter smooths out noisy data by blending the previous estimate with new data using a fixed weighting factor $\alpha$. The Kalman filter automatically adjusts its version of $\alpha$ essentially $K_k$ to decide how to weight the previous estimate and the new data for each time step depending on how much uncertainty there is in the prediction and measurement.
````

The mean square error (MSE) between $\hat{x}_k$ and $x_k$ is the performance criterion for the Kalman filter. The MSE is related to the terms in the leading diagonal of $P_k$. Here we will derive an expression for $K_k$ which minimizes the MSE. We want to obtain an expression from the definition of $P_k$ from {eq}`eq-estimation-cov` in terms of our Kalman parameters. The problem is {eq}`eq-estimation-cov` contains $e_k$ which we can't compute as we don't know $x_k$. 

First lets determine an expression for $\hat{x}_k$ by subbing {eq}`measurement` into {eq}`eq-kalman-update`, to rewrite our estimation update equation:

```{math}
:label: eq-intermediate-step
\hat{x}_k = \hat{x}^-_k + K_k(Hx_k+v_k-H\hat{x}^-_k)
```

then lets use this in our expression for $P_k$ by subbing {eq}`eq-intermediate-step` into {eq}`eq-estimation-cov`:

```{math}
:label: eq-expectation
P_k = \mathbb{E} \left\{ 
\left[ 
(x_k - \hat{x}_k^-) - K_k \left( H_k {x}_k + v_k - H_k \hat{x}_k^- \right)
\right]
\left[ 
(x_k - \hat{x}_k^-) - K_k \left( H_k {x}_k + v_k - H_k \hat{x}_k^- \right)
\right]^T
\right\}
```



````{admonition} In depth expectation simplification
By comparing {eq}`eq-expectation` and {eq}`eq-estimation-cov` we see that the estimation error $e_k$ is built from the error in our prediction and the error effect of measurement noise. The estimation error after the update step is:
```{math}
:label: eq-E1
e_k = (x_k - \hat{x}_k^-) - K_k \left( H_k {x}_k + v_k - H_k \hat{x}_k^- \right)
```
So we start with the difference between our true state and our prediction. The Kalman gain adjusts this based on what we actually measure $z_k = H_k {x}_k + v_k$ and what we are expected to measure $\hat{z} = H_k \hat{x}_k^-$ based on the model. 
We can group the terms more intuitively by substituting in the definition for the prediction error $e^-_k = x_k - \hat{x}_k^-$:
```{math}
:label: eq-E2
e_k = e^-_k \Omega_k - K_kv_k
```
Where $\Omega_k = I - K_k H_k$ describes how much of the prediction error remains after the update. To better undesrtand how uncertain out new estimate is lets return to the covariance of $e_k$ by subbing {eq}`eq-E2` into {eq}`eq-estimation-cov`:

```{math}
:label: eq-E3
P_k = \mathbb{E}[(e^-_k \Omega_k - K_kv_k)(e^-_k\Omega_k - K_kv_k)^T]
```

When we expand the brackets we get four terms:

```{math}
:label: eq-E4
P_k = \mathbb{E}[ 
\Omega_k, e^-_k (e^-_k)^T \Omega_k^T 
- \Omega_k, e^-_k v^T K_k^T 
- K_k v (e^-_k)^T \Omega^T 
+ K_k v v^T K_k^T]
```
Then using the identity $\mathbb{E}[Ax + By] \equiv A\mathbb{E}[x] + B \mathbb{E}[y]$ we can rewrite {eq}`eq-E4`
```{math}
:label: eq-E5
P_k =  
\Omega_k, \mathbb{E}[e^-_k (e^-_k)^T] \Omega_k^T 
- \Omega_k \mathbb{E}[e^-_k v^T] K_k^T
- K_k \mathbb{E}[v (e^-_k)^T] \Omega^T 
+ K_k \mathbb{E}[v v^T] K_k^T
```

The first term represents the uncertainty left from the prediction after the update. The last term represents the uncertainty added by the measurement noise. The two middle terms represent the interaction between the prediction error and the measurement noise they are both zero since $e_k$ and $v_k$ are both have mean of 0 and are independent. Subbing in {eq}`eq-prediction-cov` into the first term, {eq}`eq-measurement-noise-cov` into the last and our definition of $\Omega_k$ term gets {eq}`P_k-minimise`.
````

After performing the expectation we are left with an equation for $P_k$ in terms of Kalman filter parameters:


```{math}
:label: P_k-minimise
P_k = (I-K_kH_k)P^-_k(I-K_kH_k)^T+K_kR_kK^T_k
```

Which is a general expression for the updated error covariance matrix for suboptimal $K_k$. Now we will optimize $K_k$ by minimizing the MSE corresponding to finding the value of $K_k$ that minimizes the individual terms along the leading diagonal of $P_k$ as these terms represent the estimation error variances for the elements of the state vector elements being selected. This leaves: 

```{margin}
For a more detailed derivation see {cite}`brown2012, chapter=4`.
```

```{math}
:label: eq-kalman-gain
K_k = P^-_k H^T (H P^-_k H^T + R)^{-1}
```
In this case $K_k$ is the optimum Kalman gain. When considering the optimum Kalman gain {eq}`P_k-minimise` can be simplified to by ignoring the last two terms: 

```{math}
:label: eq-error-covariance-update
P_k = P^-_k - K_k H P^-_k
```
{cite}`brown2012, chapter=4`




Whereas the low pass filter passes $\hat{x}_{k-1}$ directly between time steps $t_{k-1}$ and $t_{k}$ the Kalman filter predicts the next step before a measurement is carried out and compares this with the measurement to give new estimate.


```{note}
$H$ and $R$ are only used in the prediction step.
```

## 2.3 Prediction step
Unlike the low pass filter Kalman filters also consider the physics of the system, the predictions which are used alongside measurements for state estimation. The system dynamically changes over time and is modelled using $A$, which describes how the state is predicted to change between $t_k$ and $t_{k+1}$:

```{math}
:label: projection
\hat{x}^-_{k+1} = A \hat{x}_k
```

The error covariance matrix associated with this prediction $\hat{x}^-_{k+1}$ is obtained from:

```{margin}
If $\mu_x$ is transformed linearly $\mu_y = F\mu_x$ its covariance matrix, $\Sigma_x$, can be transformed using $\Sigma_y = F\Sigma_xF^T$. {cite}`Barreto2021,chapter=2`
```

```{math}
:label: projection-covariance
P^-_k = AP_kA^T+Q
```

$Q$ also appears here as this is how much the true state is expected to vary from the predictions. Together these two equations encode the prediction phase of the filter.  If we only had a model we wouldn't need the estimation phase i.e. we had no measurements. 


```{note}
$A$ and $Q$ are only used in the prediction step.
```

## 2.4 Summary 

```{figure} BasicKalman.jpg
:name: fig-kalman-block-diagram
Block diagram of the Kalman filter process.
```


- $A$ and $H$ are essential for the Kalman filter if these are incorrect the filter will not converge.
- $Q$ and $R$ are used for tuning and ensure an optimum fit.
- $x_0$ and $P_0$ are initial estimates the filter will still converge even if these are wrong. 
- $K_k$ and $P_k$ are internal parameters.
- $z_k$ are the inputs (measured state)
- $\hat{x}_k$ are the outputs (estimated state)

