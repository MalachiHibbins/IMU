import numpy as np

def get(maximum, noise_level, rng, dt = 1, speed1 = 1.33, speed2=2.5, speed3 = 0.1, A1 = 1, A2 = 0.4, A3 = 0.2):
    s = np.sin(speed1*A1*np.linspace(0, maximum, int(maximum/dt))) + A2*np.sin(speed2*np.linspace(0, maximum, int(maximum/dt))) + A3*np.sin(speed3*np.linspace(0, maximum, int(maximum/dt)))  # Linear signal
    noise = rng.normal(0, noise_level, len(s))  # Gaussian noise
    signal = s + noise  # Combine signal and noise
    return signal, s