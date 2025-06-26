import numpy as np
import scipy.constants as sc
from scipy.spatial.transform import Rotation as R

def calcualte(x, p, z, A, H, Q, R, correct_mode = True):
    # Predict state error
    x_p = A@x
    P_p = A@p@A.T + Q
    
    if correct_mode:
        # Predict kalman gain
        K_k = P_p@H.T@np.linalg.inv((H@P_p@H.T+R))
        x_e = x_p + K_k@(z - H@x_p)
        P_e = P_p - K_k@H@P_p
        return x_e, P_e
    else:
        # No correction, just return predicted state and error covariance
        return x_p, P_p

def quat2EP(beta):
    """Convert quaternion to Euler parameters (EP)"""
    r = R.from_quat(beta)
    euler_angles = r.as_euler('zyx') 
    return euler_angles

def EP2quat(euler_angles):
    """Convert Euler parameters (EP) to quaternion"""
    r = R.from_euler('zyx', euler_angles) 
    quat = r.as_quat()  
    q0, q1, q2, q3 = quat[3], quat[0], quat[1], quat[2]
    return np.array([q0, q1, q2, q3])

def get_attitude_measurment(as_, ms = None):
    """Calculate the attitude measurement from accelerometer and magnetometer data."""
    a1 = as_[:, 0]
    a2 = as_[:, 1]
    # a3 is not required here
    theta = -np.arcsin(a1 / sc.g)
    phi = np.arcsin(a2 / (sc.g * np.cos(theta)))
    if ms is None:
        return np.array([theta, phi])
    else:
        mag_x = ms[:, 0] 
        mag_y = ms[:, 1]
        mag_z = ms[:, 2]
        #adjust magnetometer measurments to the horizontal
        Xh = mag_x * np.cos(theta) + mag_y * np.sin(phi) * np.sin(theta) + mag_z * np.cos(phi) * np.sin(theta)
        Yh = mag_y * np.cos(phi) - mag_z * np.sin(phi)
        psi = -np.arctan2(Yh, Xh)
        return np.array([psi, theta, phi])

def filter(as_, ws, x_i, p_i, dt = 0.05, ms = None, **kwargs):
    """Main loop for the kalman fitler"""
    n = len(ws)
    zs_e = np.zeros((n, 3))
    filtered_signal = np.zeros((n, 3))
    x = x_i
    p = p_i
    if ms is None:
        theta_as, phi_as = get_attitude_measurment(as_) # When no magnetometer data is provided, only theta and phi are calculated
    else:
        psi_ms, theta_as, phi_as = get_attitude_measurment(as_, ms = ms) # When magnetometer data is provided, psi, theta and phi are calculated
    for i in range(n):
        theta_a = theta_as[i]
        phi_a = phi_as[i]
        psi_b, theta_b, phi_b = quat2EP(x)
        if np.isnan(theta_a) or np.isnan(phi_a):
            euler = [psi_b, theta_b, phi_b]
        elif ms is not None:
            euler = [psi_ms[i], theta_a, phi_a] 
        else:
            euler = [psi_b, theta_a, phi_a]
        z = EP2quat(euler)
        w_1, w_2, w_3 = ws[i]
        B = 0.5 * np.array([
            [0, -w_1, -w_2, -w_3], 
            [w_1, 0, w_3, -w_2], 
            [w_2, -w_3, 0, w_1], 
            [w_3, w_2, -w_1, 0]
        ])
        A = np.eye(4) + dt * B
        x, p = calcualte(x, p, z, A, **kwargs)
        filtered_signal[i] = quat2EP(x)
        zs_e[i] = quat2EP(z)
    return filtered_signal, zs_e
    
    