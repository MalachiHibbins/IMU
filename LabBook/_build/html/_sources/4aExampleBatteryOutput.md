# Example: Battery output

This example looks at how a kalman filter can be used to fit a constant signal in this case the output from a battery.
- $\hat{x}$ is the voltage which is scalar hence $n = 1$, 
- $\hat{z}$ is also the voltage $m=1$
- $Q = 0$ Assuming no linear process noise ($w_k$ = 0) 
- $A=1$ the battery output is constant so $x_{k+1}$ = $x_k$
- $H = 1$ The measurement sensor directly observes the voltage
- $R$ is adjusted for tuning

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
Setting Q to anything other than $0$ in this example makes the filtered signal noisy since the filter is expecting linear process noise which doesn't exist in this model because of the way it was defined.