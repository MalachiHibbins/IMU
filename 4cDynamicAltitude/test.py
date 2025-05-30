import calibration

import matplotlib.pyplot as plt
import numpy as np

signal = calibration.get_signal(maximum=30, noise_level=0.05, rng=np.random.default_rng(seed=1), dt=0.01, speed=10)

fig, ax = plt.subplots()
ax.plot(signal, label='Signal')
ax.set_xlabel('Time')
plt.show()