import numpy as np

def get_signal(maximum, noise_level, rng, dt=1, speed=1.33, speed2=2.5, speed3=0.1, A1=1, A2=0.4, A3=0.2):
    t = np.linspace(0, maximum, int(maximum / dt))
    s = A1 * np.sin(speed * t)
    noise = rng.normal(0, noise_level, len(s))  # Gaussian noise
    signal = s + noise  # Combine signal and noise
    return signal  # Return the signals and their velocities