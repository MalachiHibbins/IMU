import GenTestSig
import SimpleKalman

import matplotlib.pyplot as plt
import numpy as np

length = 100
value = 25
std = 0.5
rng = np.random.default_rng(seed=1)
A = 1.0
H = 1.0
Q = 0
R = 4
x_e = 24
P_0 = 2

test_signal = GenTestSig.get(value, rng, std=std, size=length)
filtered_signal = SimpleKalman.filter(test_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
ax1.scatter(np.arange(length), test_signal, label='Noisy Signal', color='blue', alpha=0.1)
ax1.plot(filtered_signal, label='Kalman Filtered Signal', color='orange')
ax1.hlines(value, 0, length, colors='green', linestyles='dashed', label='True Value')
ax1.set_xlabel('Sample Index')
ax1.set_ylabel('Value')
ax1.set_title('Kalman Filter Example')
ax1.legend()

ax2.plot(abs(filtered_signal-value), label='Filtered Signal - True Value', color='orange')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Value Difference')
ax2.set_title('Difference from True Value')
ax2.legend()
plt.show()

# print final value and error
final_value = filtered_signal[-1]
error = abs(final_value - value)
print(f"Final Value: {final_value}, Error: {(error/value)*100}%")