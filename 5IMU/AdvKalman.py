import numpy as np
import scipy.constants as sc

def calcualte(x, p, z, A, H, Q, R):
    # Predict state error
    x_p = A@x
    P_p = A@p@A.T + Q
    
    # Predict kalman gain
    K_k = P_p@H.T@np.linalg.inv((H@P_p@H.T+R))
    x_e = x_p + K_k@(z - H@x_p)
    P_e = P_p - K_k@H@P_p
    return x_e, P_e

def EP2euler(beta):
    q0, q1, q2, q3 = beta
    phi = np.arctan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)
    theta = np.arcsin(-2*(q1*q3-q0*q2))
    psi = np.arctan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)
    return np.array([psi, theta, phi])

def euler2EP(euler_angles):
    psi, theta, phi = euler_angles
    q0 = np.cos(phi/2) * np.cos(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    q1 = np.sin(phi/2) * np.cos(theta/2) * np.cos(psi/2) - np.cos(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    q2 = np.cos(phi/2) * np.sin(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.cos(theta/2) * np.sin(psi/2)
    q3 = np.cos(phi/2) * np.cos(theta/2) * np.sin(psi/2) - np.sin(phi/2) * np.sin(theta/2) * np.cos(psi/2)
    return np.array([q0, q1, q2, q3])

def a2euler(as_):
    a1 = as_[:, 0]
    a2 = as_[:, 1]
    # a3 = as_[:, 2]  # Not used for theta/phi here

    theta = np.arcsin(a1 / sc.g)
    phi = -np.arcsin(a2 / (sc.g * np.cos(theta)))
    psi = np.zeros_like(theta)
    return np.array([psi, theta, phi])

def filter_no_fusion(ws, x_i, p_i, dt, **kwargs):
    n = len(ws)
    filtered_signal = np.zeros((n, 3))
    x = x_i.copy()
    p = p_i.copy()
    
    for i in range(1, n):
        w_1, w_2, w_3 = ws[i]
        B = 0.5 * np.array([
            [0, -w_1, -w_2, -w_3], 
            [w_1, 0, w_3, -w_2], 
            [w_2, -w_3, 0, w_1], 
            [w_3, w_2, -w_1, 0]
        ])
        A = np.eye(4) + dt * B
        x, p = calcualte(x, p, x, A, **kwargs)
        
        filtered_signal[i] = EP2euler(x)
    return filtered_signal

def filter(as_, ws, x_i, p_i, dt, **kwargs):
    n = len(ws)
    filtered_signal = np.zeros((n, 3))
    x = x_i.copy()
    p = p_i.copy()
    eulers = a2euler(as_).T  # shape (n, 3)
    
    zs = np.apply_along_axis(euler2EP, 1, eulers)  # shape (n, 4)
    for i in range(n):
        w_1, w_2, w_3 = ws[i]
        B = 0.5 * np.array([
            [0, -w_1, -w_2, -w_3], 
            [w_1, 0, w_3, -w_2], 
            [w_2, -w_3, 0, w_1], 
            [w_3, w_2, -w_1, 0]
        ])
        A = np.eye(4) + dt * B
        x, p = calcualte(x, p, zs[i], A, **kwargs)
        filtered_signal[i] = EP2euler(x)
    return filtered_signal
    
    