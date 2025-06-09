# Determining Q
Q_k is the covariance matrix of $w_k$ defined as:
```{math}
\begin{split}E[w_k w_i^T] = 
\begin{cases}
    Q_k, & i = k \\
    0,   & i \neq k
\end{cases}\end{split}
```
Q is difficult to determine 