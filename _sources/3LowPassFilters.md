# Low Pass filters

A low pass filter is an improvement on the moving average filter it allows low frequencies to pass through but filters out high frequencies. Noise is usually high frequency. Below is an example of a first order low pass filter.

```{math}
:label: eq-lowpass-1
\hat{x}_k = \alpha \hat{x}_{k-1} + (1 - \alpha) z_k \quad 0<\alpha<1
```

Looks like expression (1.3) except here $\alpha$ is a free parameter to be chosen. It is also true that:

```{math}
:label: eq-lowpass-2
\hat{x}_{k-1} = \alpha \hat{x}_{k-2} + (1 - \alpha) z_{k-1} \quad 0<\alpha<1
```

Combining these equations helps to overcome some problems associated with the moving average since more distant terms disappear exponentially quickly:

```{math}
:label: eq-lowpass-3
\hat{x}_k = \alpha^2 \hat{x}_{k-2} + \alpha(1-\alpha) z_{k-1} + (1-\alpha)z_k \quad 0<\alpha<1
```

Due to the restriction on alpha larger $n$ means greater weighting on $\hat{x}_n$ since $\alpha(1-\alpha)\leq 1-\alpha$. Previous data gets weighted exponentially less. 

```{figure} image-10.png
:name: fig-lowpass-vs-moving-average
Figure 3.1: On the left is the moving average filter from {numref}`fig-moving-average-k25`, on the right is the low pass filter with optimized $\alpha = 0.9$ (by eye).
```
The low pass filter has a smaller delay compared to the moving average filter. However the low pass filter didn't remove as much noise as the moving average filter did.

```{figure} image-13.png
:name: fig-lowpass-alpha-08
Figure 3.2: Low pass filter with $\alpha = 0.8$.
```
Here the delay is less significant but the filter doesn't remove as much noise as it did when $\alpha = 0.9$, since the moving average is more easily influenced by more recent measurements.

```{figure} image-14.png
:name: fig-lowpass-alpha-095
Figure 3.3: Low pass filter with $\alpha = 0.95$.
```
Here the delay is more significant since results are given more similar weightings, hence figure 3.3 looks most similar to the moving average filter but with a smaller delay. The choice of $\alpha$ represents a tradeoff between a noisy signal and a delayed signal, meaning there is an optimal choice of $\alpha$ exists ensuring that the overall effects of both are minimized.

```{admonition} Idea
For purpouses such as numerical integration is it possible to calculate the delay and then change the bounds of the intergral rather than trying to fit the data with a less smooth curve. 
```