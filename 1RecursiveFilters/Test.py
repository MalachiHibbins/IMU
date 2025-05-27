import GetVolt
import RcsAvgFilter
import numpy as np
import matplotlib.pyplot as plt

value = 5.0
size = 30
rng = np.random.default_rng(seed=40)
rolling_avg = []

# Generate a noisy signal
signal = GetVolt.GetVolt(value, rng, std=0.5, size=size)

# Apply the recursive average filter
rolling_avg = RcsAvgFilter.filter(signal)

# Plot results
fig, ax = plt.subplots()
ax.scatter(np.arange(size), signal, label='Noisy Signal', color='blue')
ax.plot(rolling_avg, label='Average', color='orange')
ax.hlines(value, 0, size, colors='green', linestyles='dashed', label='True Value')
ax.set_xlabel('Sample Index')
ax.set_ylabel('Value')
ax.set_title('Rolling Average Filter')
ax.legend()
plt.show()
