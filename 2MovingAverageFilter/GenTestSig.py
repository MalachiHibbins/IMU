import numpy as np

def generate_test_signal(length, noise_level, rng, frequency_of_points = 10, speed1 = 1.33, speed2=2.5, speed3 = 0.1, A1 = 1, A2 = 0.4, A3 = 0.2):
    s = A1*np.sin(speed1*np.linspace(0, frequency_of_points, length)) + A2*np.sin(speed2*np.linspace(0, frequency_of_points, length)) + A3*np.sin(speed3*np.linspace(0, frequency_of_points, length))  # Linear signal
    noise = rng.normal(0, noise_level, length)  # Gaussian noise
    signal = s + noise  # Combine signal and noise
    return signal, s