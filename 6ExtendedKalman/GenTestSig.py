import numpy as np

def get(maximum, noise_level, rng, noise_level_a = 0.02, dt = 1, speed1 = 1.33, speed2=2.5, speed3 = 0.1, A1 = 1, A2 = 0.4, A3 = 0.2):
    t = np.linspace(0, maximum, int(maximum/dt)) 
    s = A1*np.sin(speed1*t) + A2*np.sin(speed2*t) + A3*np.sin(speed3*t)  # Linear signal
    v = speed1*A1*np.cos(speed1*t)  + A2*speed2*np.cos(speed2*t) + A3*speed3*np.cos(speed3*t)  # Velocity signal
    a = -speed1**2*A1*np.sin(speed1*t) - A2*speed2**2*np.sin(speed2*t) - A3*speed3**2*np.sin(speed3*t)  # Acceleration signal
    noise = rng.normal(0, noise_level, len(s))  # Gaussian noise
    noise_a = rng.normal(0, noise_level, len(a))  # Gaussian noise for acceleration
    signal_s = s + noise  # Combine signal and noise
    signal_v = np.diff(signal_s, prepend = 0) / dt
    signal_a = a + noise_a # Differentiate to get acceleration signal
    return signal_s, s, signal_v, v, signal_a, a  # Return the signals and their velocities