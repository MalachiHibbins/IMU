# Average filters

The estimated state (the mean om this example) is evaluated as:

```{math}
:label: eq-average
\hat{x}_k = \frac{z_1+z_2+z_3+...+z_k}{k}
```

Which can be written recursively as:

```{math}
:label: eq-recursive
\hat{x}_k = \left(\frac{k-1}{k}\right)\hat{x}_{k-1} + \frac{z_k}{k}
```
which is more efficient to compute.

Let $\alpha = \frac{k-1}{k}$, equation {eq}`eq-recursive` can be rewritten as:

```{math}
:label: eq-alpha
\hat{x}_k = \alpha \hat{x}_{k-1} + (1 - \alpha) z_k
```

This is an example of a recursive (average) filter. 

```{figure} image-8.png
:name: fig-noisy-signal
_A noisy signal with mean 25 and standard deviation 0.5, fitted with a moving average filter._
```

The filter is efficient and converges to the correct value very quickly. Due to random variation the filter fluctuates around the correct value. For obvious reasons this filter will not work for a constantly varying signal.