# Kalman Filters 

How they work and why you should use them.
*Malachi Hibbins*

---

## Introduction
- Kalman filters are an optimal estimation algorithm which looks at noisy measurements and attempts to predict the true state 
- A Kalman filter considers:
- **The model**: *Predicts* the next state from the current state *estimate*.
- **The measurement** containing noise
- And combines these to form an *updated estimate* of the state.

---
## Prediction 

- The true state, $x_k$, evolves according to the linear model defined in $A$:
$$x_{k+1} = Ax_k + w_k$$
- $w_k$ is white noise associated with the linear process. 
- We don't have access to $x_k$ or only its estimate $\hat{x}_k$. 
- We can predict the state:
$$\hat{x}^-_{k+1} = A \hat{x}_k$$
- And its associated error covariance matrix:
$$P^-_k = AP_kA^T+Q$$
- $P_k$ and $P^-_k$ are error covariance matrices for $\hat{x}_k$ and $\hat{x}^-_k$
- $Q$ is the error covariance matrix for $w_k$ 
