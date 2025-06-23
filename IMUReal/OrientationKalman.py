import AdvKalman
import numpy as np
import Integrate


def run(ws, as_, q = 10**-1.6, r = 10**0.7, p = 0.1, dt = 0.05):
    # Simulation parameters
    Q = np.identity(4) * q
    R = np.identity(4) * r
    p_i = np.identity(4) * p  # initial covariance matrix
    H = np.identity(4)
    x_i = np.array([1, 0, 0, 0]) # initial state vector
    
    # Use the Kalman filter to fuse the gyroscope and accelerometer data
    filtered_signal = AdvKalman.filter(as_, ws, x_i, p_i, dt, H=H, Q=Q, R=R)
    
    # Calculate the euler parameters from the accelerometer data
    eulers_a = AdvKalman.get_attitude_measurment(as_)
    print(eulers_a)
    
    # Calculate the euler angles using euler integration
    eulers_g = AdvKalman.filter(as_, ws, x_i, p_i, dt, H=H, Q=Q, R=R, correct_mode=False)
    

    return filtered_signal, eulers_a, eulers_g





