# Position using GPS and Accelarometer data
Here we will improve on the [the velocity from position example](4bExampleVelocityFromPosition.md) example using sensor fusion. While the system using only position data (which we measumre using GPS) works well for predicting the position, its not so good at predicting the velocity. The current model {eq}`eq-motion-equations` represents an oversimplification as it assumes no accelaration (the accelaration doesn't change between steps) which means that the velocity has to be corrected for by the measurments which is what causes lag. We could improve our model using real world accelarometer data which we can integrate to find velocity and position. There are other reasons for including the accelarometer data for example when GPS isn't available due to some form of blocking e.g. being in a tunnel, the device can still roughly determine its position.  

## Model
To begin with we will work with the simple 1D case. Our new model is built on {eq}`eq-motion-equations` with an additional 2nd order term:

```{math}
:label: eq-motion-equations2
s_{k+1} &\approx s_k + \nu_k\Delta t + \frac{1}{2}a_k \Delta t^2\\
\nu_{k+1} &\approx \nu_k + a\Delta t
```

The parameters from [the velocity from position model](4bExampleVelocityFromPosition.md#model) remain the same except for $A$ which changes because the model changes.
- $\hat{\boldsymbol{x}}$ is the column vector of position and velocity
- $z$ is the measurment of position from the GPS
- $H = \begin{bmatrix} 1 & 0 \end{bmatrix}$
- $Q = \sigma_a^2 \begin{bmatrix} \frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\ \frac{\Delta t^3}{2} & \Delta t^2 \end{bmatrix}$ Where $\sigma_a$ will be the standard deviation in the accelaration measurments.
- $R = \sigma_s^2$ Where $\sigma_s$ will be the standard deviation in the position measurments.
We need $A$ to be a $2 \times 2$ matrix, since it has the same number of rows and columns as the number of entries in $\hat{z}$, but this isn't possible since {eq}`eq-motion-equations2` contains 3 terms. 

```{margin}
If $\mu_x$ is transformed linearly $\mu_y = F\mu_x$ its covariance matrix, $\Sigma_x$, can be transformed using $\Sigma_y = F\Sigma_xF^T$. {cite}`Barreto2021,chapter=2`
```

````{admonition} Extended Kalman Filters
So we can't write our equations of motion in the correct for form for the simple kalman filter. This is because the equations of motion {eq}`eq-motion-equations2` in this case aren't linear. The extended kalman filter predicts the next state using:
```{math}
:label: eq-proj-ext
\hat{\boldsymbol{x}}^-_{k+1} = A\hat{\boldsymbol{x}}_k + B\boldsymbol{u}_k
```
Where $u_k$ is the forcing function and $B$ is its associated control matrix where $u_k$ is the forcing function and $B$ is its associated control matrix.



```{math}
:label: eq-proj-cov-ext
P^-_{k+1} = A_kP_kA_k^T+B_kR^uB_k^T+Q
```
Where $R^u$ is the associated error covariance matrix for $u_k$. Equation {eq}`eq-proj-cov-ext` is the updated form of {eq}`eq-error-covariance-update` with the final term $B_kR^uB_k^T$ corresponding to the covariance update for $R^u$. $R^u$ becomes one of our kalman parameters when using the extended kalman filter.
```{figure} ExpandedKalman.jpg
:name: fig-block-kalman
Block diagram for the extended kalman filter. The estimation phase is equivalent to {numref}`fig-kalman-block-diagram` but the prediction stage has been updated.
```
````

So lets re-write {eq}`eq-motion-equations2` in the form of {eq}`eq-proj-cov-ext` to determine $u_k$ and $B$.

```{math}
\begin{bmatrix} s \\ \nu \end{bmatrix}^-_{k+1} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} s \\ \nu \end{bmatrix}_k + \begin{bmatrix} \Delta t^2 \\ \Delta t \end{bmatrix} a_k.
```

Which gives $u_k = \begin{bmatrix} \Delta t^2 \\ \Delta t \end{bmatrix}$ and $u_k = a_k$. Sine we are working in one dimension $R^u = \sigma_a'^2$ which we will determine by tuning.

```{figure} image-31.png
:name: fig-improved-vel-pos
Velocity and position as a functon of time plotted for the extended kalman filter using the same parameters in {numref}`fig-increased-R-and-increased-Q`, tuned by eye. [View in Github](https://github.com/MalachiHibbins/IMU/tree/main/6ExtendedKalman)
```

Compared to {numref}`fig-increased-R-and-increased-Q` the externded kalman fitler with acceleration measurments gives a better fit for position and a significantly better fit for velocity, helped by the significantly better model. Even without measurment corrections the accelarometer gives a supprisingly good fit although there is a tiny bit of drift visible at the end. However the drift is significantly larger when integrated twice. 
 
## Smartphone example
Now we will consider a more realistic example of predicting 1D motion simulating sensors found in mobile phones. Mobile phones are equipt with low performance micro electrical mechanical systems (MEMS) from which we can implement IMU combined with correcting GPS data can form an intertial measurment system (INS).

**Accelarometer**. A typical mobile phone accelarometer takes readings at a rate of between $10$ Hz $6664$ Hz here we will assume about $400$ Hz {cite}`Grouios2022`, has an error of approximately $0.01$ ms$^{-1}$ {cite}`Capuano2023, section=3`. Accelarometers become less reliable over time, there noise over time is desribed by random walks, hence they need to be corrected for by GPS.

**GPS** A typical GPS accearometer takes readings at a rate of $1$ Hz and has an error of approximately $5$ m but this vary changable due to geographical factors. Futhermore in some case GPS is bocked entirely. GPS is relied apon for correcting measurments as the noise is uncorrelated over time.

Lets look 


