# Moving Average filters

The moving average $\hat{x}_n$ is used to remove noise over a constantly varying signal. Here $n$ represents the index of the moving average and $k$ represents the window size which is a free parameter.

```{math}
:label: eq-moving-average
\hat{x}_n = \frac{z_{n-k+1} + z_{n-k+2}+...+z_n}{k}
```

$\hat{x}_n$ can be written recursively as: 

```{math}
:label: eq-moving-average-rec
\hat{x}_n = \hat{x}_{n-1} + \frac{z_n - z_{m-k}}{k} 
```

```{figure} image-9.png
:name: fig-moving-average-k25
Moving average filter applied to a noisy signal with $k = 25$.
```

The moving average lags behind the true signal but has roughly the right shape.

```{figure} image-15.png
:name: fig-moving-average-k5
Moving average filter applied to a noisy signal with $k=5$.
```

In this example the delay is smaller but less noise is removed. There is a tradeoff between noise reduction and minimizing delay limiting the reliability of the moving average filter. 
All terms in {eq}`eq-moving-average` have equal weighting ($1/n$). However it makes more sense to give more recent terms a larger weighting.