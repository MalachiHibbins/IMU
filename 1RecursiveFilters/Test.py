import GetVolt
import RcsAvrFilter
import numpy as np


value = 5.0
size = 30
rng = np.random.default_rng(seed=42)


signal = GetVolt.GetVolt(value, rng, std=0.5, size=size)
for s in signal:
    rolling_avg = s