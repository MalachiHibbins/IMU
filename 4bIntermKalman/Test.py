import AdvKalman
import GenTestSig

import matplotlib.pyplot as plt
import numpy as np

maximum = 10
std = 0.1
rng = np.random.default_rng(seed=1)
dt = 0.01

A = np.array([[1, dt], [0, 1]]) 
H = np.array([[1, 0]])
Q = np.array(([[1,0], [0, 3]])) 
R = 20
x_e = np.array([0, 2])  # Initial state estimate
P_0 = np.array([[5.0, 0], [0, 5.0]])  # Initial error covariance


noisy_signal, signal = GenTestSig.get(maximum, std, rng, dt=dt)
filtered_signal = AdvKalman.filter(noisy_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)

s_k = filtered_signal[:, 0]
s_v = filtered_signal[:, 1]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
ax1.scatter(np.arange(len(noisy_signal)), noisy_signal, label='Noisy Signal', color='blue', alpha=0.1)
ax1.plot(signal, label='True Signal', color='green')
ax1.plot(s_k, label='Kalman Filtered Position', color='orange', alpha=0.7)
ax1.set_xlabel('Sample Index')
ax1.set_ylabel('Position')
ax1.set_title('Kalman Filter Example')
ax1.legend()

ax2.plot(s_v, label='Kalman Filtered Velocity', color='orange')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Velocity')
ax2.set_title('Kalman Filtered Velocity')
ax2.legend()
plt.tight_layout()
plt.show()