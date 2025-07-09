import LowPassFilter
import GenTestSig

import matplotlib.pyplot as plt
import numpy as np

length= 1000
noise = 0.5
rng = np.random.default_rng(seed=45)
window_size = 25
frequency_of_points = 5
speed1 = 1.33
speed2 = 7
speed3 = 16
A1 = 1
A2 = 0.4
A3 = 0.2
alpha = 0.8

# Generate a test signal with noise
signal_noisy, signal_true = GenTestSig.generate_test_signal(length, noise, rng, frequency_of_points=frequency_of_points,
                                                              speed1=speed1, speed2=speed2, A1=A1, A2=A2, speed3=speed3, A3=A3)
# Apply the moving average filter
moving_avg = LowPassFilter.filter(signal_noisy, alpha)

fig, ax = plt.subplots()
ax.scatter(np.arange(length), signal_noisy, label='Noisy Signal', color='blue', alpha=0.1)
ax.plot(signal_true, label='True Signal', color='green', linestyle='--')
ax.plot(moving_avg, label='Low Pass Filter', color='orange')
ax.legend()
ax.set_xlabel('Sample Index')
ax.set_ylabel('Value')
plt.show()