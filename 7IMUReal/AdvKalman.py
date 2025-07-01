import numpy as np
import scipy.constants as sc
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def difference(z, H, x_p):
    # Compute predicted measurement
    z_pred = H @ x_p
    
    # Compute raw difference
    diff = z - z_pred
    
    # Handle quaternion double cover (q and -q represent same rotation)
    # Choose the quaternion that gives the smaller angle difference
    if np.dot(z, z_pred) < 0:
        # If dot product is negative, use -z_pred instead
        diff = z - (-z_pred)
    
    return diff

def calcualte(x, p, z, A, H, Q, R, correct_mode = True):
    # Predict state error
    x_p = A@x
    P_p = A@p@A.T + Q
    
    if correct_mode:
        # Predict kalman gain
        K_k = P_p@H.T@np.linalg.inv((H@P_p@H.T+R))
        x_e = x_p + K_k @ difference(z, H, x_p)
        P_e = P_p - K_k@H@P_p
        return x_e, P_e
    else:
        # No correction, just return predicted state and error covariance
        return x_p, P_p

def quat2EP(beta):
    """Convert quaternion to Euler parameters (EP)"""
    q0, q1, q2, q3 = beta
    psi = np.arctan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)
    phi = np.arctan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)
    sin_theta = -2 * (q1 * q3 - q0 * q2)  # sin(theta) = -2*(q1*q3 - q0*q2)
    if abs(sin_theta) <= 1:
        # Normal case
        theta = np.arcsin(sin_theta)
    else:
        # Handle cyclic case - when |sin_theta| > 1
        if sin_theta > 1:
            # sin_theta > 1 means theta should be π/2 + extra rotation
            theta = np.pi/2 - np.arcsin(2 - sin_theta)
        else:  # sin_theta < -1
            # sin_theta < -1 means theta should be -π/2 - extra rotation  
            theta = -np.pi/2 + np.arcsin(2 + sin_theta)
    theta = (theta - np.pi/2) % np.pi - np.pi/2  # Normalize to [-pi/2, pi/2]
    return np.array([psi, theta, phi])
    
    
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
    # a3 = as_[:, 2]  # Not used for theta/phi here
    theta = -np.arcsin(np.clip(a1 / sc.g, -1, 1))  # Clip to avoid NaN from arcsin
    phi = np.arcsin(np.clip(-a2 / (sc.g * np.cos(theta)), -1, 1))  # Clip to avoid NaN from arcsin
    theta = theta - theta[0]  # Normalize to start at zero
    phi = phi - phi[0]  # Normalize to start at zero
    if ms is None:
        psi = np.zeros_like(theta)
        return np.array([psi, theta, phi])
    else:
        mag_x = ms[:, 0] 
        mag_y = ms[:, 1]
        mag_z = ms[:, 2]
        #adjust magnetometer measurments to the horizontal
        Xh = mag_x * np.cos(theta) + mag_y * np.sin(theta) * np.sin(phi) + mag_z * np.sin(theta) * np.cos(phi)
        Yh = mag_y * np.cos(phi) - mag_z * np.sin(phi)

        psi_ = np.arctan2(Yh, Xh)
        psi = (((psi_ + np.pi) - psi_[0]) % (2 * np.pi)) - np.pi  # Normalize to [-pi, pi]
        return np.array([-psi, -theta, phi])

def filter(as_, ws, x_i, p_i, dt = 0.05, ms = None, **kwargs):
    """Main loop for the kalman fitler"""
    n = len(ws)
    filtered_signal = np.zeros((n, 3))
    x = x_i.copy()
    p = p_i.copy()
    eulers = get_attitude_measurment(as_, ms = ms).T
    zs_q = np.apply_along_axis(EP2quat, 1, eulers)  # shape (n, 4)
    
    for i in range(n):
        w_1, w_2, w_3 = ws[i]
        B = 0.5 * np.array([
            [0, -w_1, -w_2, -w_3], 
            [w_1, 0, w_3, -w_2], 
            [w_2, -w_3, 0, w_1], 
            [w_3, w_2, -w_1, 0]
        ])
        A = np.eye(4) + dt * B
        x, p = calcualte(x, p, zs_q[i], A, **kwargs)
        filtered_signal[i] = quat2EP(x)
    zs_EP = np.apply_along_axis(quat2EP, 1, zs_q)  # shape (n, 3)
    print(filtered_signal)
    return filtered_signal, zs_EP
    
    