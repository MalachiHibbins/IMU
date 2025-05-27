import GenTestSig
import SimpleKalman

import matplotlib.pyplot as plt
import numpy as np

length = 1000
value = 25
std = 0.5
rng = np.random.default_rng(seed=45)
A = 1.0
H = 1.0
Q = 0
R = 4
x_e = 6
P_0 = 2

test_signal = GenTestSig.get(value, rng, std=std, size=length)
filtered_signal = SimpleKalman.filter(test_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)

fig, ax = plt.subplots()
ax.scatter(np.arange(length), test_signal, label='Noisy Signal', color='blue', alpha=0.1)
ax.plot(filtered_signal, label='Kalman Filtered Signal', color='orange')
ax.hlines(value, 0, length, colors='green', linestyles='dashed', label='True Value')
ax.set_xlabel('Sample Index')
ax.set_ylabel('Value')
ax.set_title('Kalman Filter Example')
ax.legend()
plt.show()

