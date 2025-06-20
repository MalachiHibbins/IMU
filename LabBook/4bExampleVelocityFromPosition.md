# Velocity from position

## Using only velocity data
This example looks at applying a kalman filter to estimate the true position and velocity from noisy position data. 

### Model
In this case $\hat{\boldsymbol{x}}_k = \begin{bmatrix} s_k \\ \nu_k \end{bmatrix}$ where $s_k$ and $\nu_k$ are the position and velocity (in the same direction) respectibly at time $t_k$.  The measurment of the state only measures position therefore $z_k = s_k$. The measurment and the state are related by {eq}`eq-h-calculate` from this we can calculate $H$.
```{math}
:label: H
z_k &= H \begin{bmatrix} s_k \\ \nu_k \end{bmatrix} \\
\implies H &= \begin{bmatrix} 1 & 0 \end{bmatrix}
```
For our model we will assume that there is no forcing function hence accelaration will be constant. This allows us to use equations of constant motion:
```{math}
:label: eq-motion-equations
s_{k+1} &\approx s_k + \nu_k\Delta t \\
\nu_{k+1} &\approx \nu_k
```
```{admonition} why use $\approx$ not $=$
Although the error from numerical integration is negligable term to term over a large numer of terms such as in this example the error accumulates. 
```
Where $\Delta t = t_{k+1} - t_k$. We now need to write our update equations in a form which can be used by the kalman fitler {eq}`projection`:
```{math}
:label: velocity
\begin{bmatrix} s \\ \nu \end{bmatrix}^-_{k+1} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} s \\ \nu \end{bmatrix}_k
```

Q is a bit more complicated to work out and is going to be a $2 \times 2$ matrix of the form below:

```{math}
Q = E[w_kw_k^T] =  \begin{bmatrix} VAR(s) & COV(s,\nu) \\ COV(\nu,s) & VAR(\nu) \end{bmatrix} = \sigma_a^2 \begin{bmatrix} \frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\ \frac{\Delta t^3}{2} & \Delta t^2 \end{bmatrix}
```
{cite}`Barreto2021,chapter=2.3` {cite}`Barreto2021,chapter=10.2`
where $\sigma_a^2$ is the variance in the true accelaration, which is the model assumes is zero. $\sigma^2$ will be used as a tuning parameter.
$R = VAR(z_k)$ is going to be the variance in the measurments of the position which we will denote as $\sigma_s$ {cite}`Barreto2021,chapter=10.2`.

We will use the follwing parametres for the kalman filter.
- $\hat{\boldsymbol{x}}_k = \begin{bmatrix} s_k \\ \nu_k \end{bmatrix}$
- $z_k = s_k$
- $A = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$
- $H = \begin{bmatrix} 1 & 0 \end{bmatrix}$ 
- $Q$ and $R$ will are tuned using the parameters $\sigma_a$ and $\sigma_s$.
- $\boldsymbol{\hat{x}}_0 = \boldsymbol{0}$ initially and $P_0 \approx \begin{bmatrix} P_s & 0 \\ 0 & P_v\end{bmatrix}$ initially, these don't matter so much as the kalman filter will eventually find the correct values as $k$ increases.

```{note}
While optimal $P_0$ isn't necessarily a diagonal matrix its good enough for an initial approximation and tuned easily using the two parameters.
```

### Implementation


The true signal in the graphs below is generated using a combination of $\sin$ waves of varing amplitudes and speeds, the velocity version is the analytical deriative of this. The signal we are trying to use the kalman filter to fit is generated by adding random noise to the signal we generated and its analytical derivative. The kalman filter is applied to the noisy position data and outputs the filtred postion data and filtered velocity data. Which are graphed below.
The $r^2$ value mean squared error (MSE) and mean absolute error (MAE) are given for both velocity and position.


```{figure} image-23.png
:name: fig-original
The measured noisy $s_k$, noise free $s_k$ and kalman filtered $s_k$ plotted against index and kalman filtered $\nu_k$ noise free $\nu_k$ and the differentiated kalman filtered $s_k$ against index. [View on GitHub](https://github.com/MalachiHibbins/IMU/tree/main/4bIntermKalman)
```

The fit is bad for both graphs is very bad and the filtered signal is lagging behind the true signal. This is because too much emphasis is being put on the measured values compared to the predicted values. Specifically kalman filter is relying too much on the model for velocity, which suggests velocity remains the same from one time step to the next which is erronous. This means the velocity is consistantly delayed so the position will also be delayed. Lets fix this by reducing the amount we trust the model, increase $Q$ (by increasing $\sigma_a$), and increasing the amount we trust the measurments, reduce $R$ (by decreasing $\sigma_s$).

```{figure} image-24.png
:name: fig-increased-R-and-increased-Q
See {numref}`fig-original` with increased $Q$ and decreased $R$. [View on GitHub](https://github.com/MalachiHibbins/IMU/tree/main/4bIntermKalman)
```

Figure {numref}`fig-increased-R-and-increased-Q` shows a slightly improved fit with position and a significantly improved fit with velocity. This makes sense for the postion data since decreasing $R$ increases the weighting for the measurment. However the velocity fit is now significantly less smooth since more emphasis is being put on the measured positions which is always noiser than the prediction. There is a clear tradeoff between having a a smooth fit and having an accurate fit. Smoother fits more accurately represent the true shape of the data, but will often cause there may be a delay or drift. Both filtered sets have random residuals, indicating they are ga good fit.

```{important}
 If the Kalman filter fit is too noisy, due to an over emhasis on $\hat{z}_k$ being used to calculate $\hat{x}_{k+1}$ increasing $R$ or decreasing $Q$ will make the fit smoother. If the kalman filter fit is delayed increasing $Q$ or decreasing $R$ will reduce the delay. Increasing $Q$ has essentially the same effect as decreasing $R$ and increasing $R$ has the same effect as decreasing $Q$. 
``` 

```{admonition} Question
What happens if we implement a model that instead of relying on constant accelaration relies on constant jerk.
```



Here it's clear the differentiated kalman position is a poor fit as its too noisy, but the kalman filtered velocity fits like a low pass filter. **A kalman filter compares predicted values with measurments, which are weighted in a similar fasion to the low pass filter, to create an estimate of the true state measurments**. The fit for $\nu_k$ would be greatly improved using a second kalman filter which would use the velocity from this kalman filter as a prediction and compare it with real velocity data.

