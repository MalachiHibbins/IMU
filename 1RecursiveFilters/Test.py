import GetVolt
import RcsAvgFilter
import numpy as np
import matplotlib.pyplot as plt

value = 25
size = 100
std = 0.5
rng = np.random.default_rng(seed=42)
rolling_avg = []

# Generate a noisy signal
signal = GetVolt.GetVolt(value, rng, std=std, size=size)

# Apply the recursive average filter
rolling_avg = RcsAvgFilter.filter(signal)

# Plot results
fig, ax = plt.subplots()
ax.scatter(np.arange(size), signal, label='Noisy Signal', color='blue', alpha=0.1)
ax.plot(rolling_avg, label='Average', color='orange')
ax.hlines(value, 0, size, colors='green', linestyles='dashed', label='True Value')
ax.set_xlabel('Sample Index')
ax.set_ylabel('Value')
ax.set_title('Rolling Average Filter')
ax.legend()
plt.show()

# print final value and error
final_value = rolling_avg[-1]
error = abs(final_value - value)
print(f"Final Value: {final_value}, Error: {error/value*100}%")
