import AdvKalman
import numpy as np

def set_zero(signal):
    # Subtract the initial value from each column
    yaw = signal[:, 0] - signal[0, 0]
    pitch = signal[:, 1] - signal[0, 1]
    roll = signal[:, 2] - signal[0, 2]

    # Normalize to correct domains
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi           # [-pi, pi]
    pitch = np.clip(pitch, -np.pi/2, np.pi/2)           # [-pi/2, pi/2]
    roll = (roll + np.pi) % (2 * np.pi) - np.pi         # [-pi, pi]
    
    return np.column_stack((yaw, pitch, roll))

def run(ws, as_, q = 10**-1.6, r = 10**-1.6, p = 0.1, ms = None, **kwargs):
    # Simulation parameters
    Q = np.identity(4) * q
    R = np.identity(4) * r
    p_i = np.identity(4) * p  # initial covariance matrix
    H = np.identity(4)
    x_i = np.array([1, 0, 0, 0]) # initial state vector
    
    
    # Use the Kalman filter to fuse the gyroscope and accelerometer data
    filtered_signal, zs = AdvKalman.filter(as_, ws, x_i, p_i, H=H, Q=Q, R=R, correct_mode=True, ms=ms, **kwargs)

    # Calculate the euler parameters from the accelerometer data
    eulers_a = AdvKalman.get_attitude_measurment(as_, ms=ms)
    
    # Calculate the euler angles using euler integration
    eulers_g, _ = AdvKalman.filter(as_, ws, x_i, p_i, H=H, Q=Q, R=R, correct_mode=False, ms=ms, **kwargs)


    return filtered_signal, eulers_a, eulers_g, zs





