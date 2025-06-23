import numpy as np
import scipy.constants as sc

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

def EP2euler(beta):
    q0, q1, q2, q3 = beta
    psi = np.arctan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)
    theta = np.arcsin(-2*(q1*q3-q0*q2))
    phi = np.arctan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)
    return np.array([psi, theta, phi])

def euler2EP(euler_angles):
    psi, theta, phi = euler_angles
    q0 = np.cos(phi/2) * np.cos(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    q1 = np.sin(phi/2) * np.cos(theta/2) * np.cos(psi/2) - np.cos(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    q2 = np.cos(phi/2) * np.sin(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.cos(theta/2) * np.sin(psi/2)
    q3 = np.cos(phi/2) * np.cos(theta/2) * np.sin(psi/2) - np.sin(phi/2) * np.sin(theta/2) * np.cos(psi/2)
    return np.array([q0, q1, q2, q3])

def get_attitude_measurment(as_, cutoff = 0.99):
    a1 = as_[:, 0]
    a2 = as_[:, 1]
    # a3 = as_[:, 2]  # Not used for theta/phi here  
    ag = a1 / sc.g
    ag_ = np.where((ag >= -cutoff) & (ag <= cutoff), ag, np.nan)
    theta = np.arcsin(ag_)
    phi = np.arcsin(np.clip((-a2 / (sc.g * np.cos(theta))), -1, 1))
    #theta = np.arctan(a1 / a3)
    #phi = -np.arctan(a2 / (a2**2 + a3**2)**0.5)
    
    return np.array([theta, phi])

def filter(as_, ws, x_i, p_i, dt, **kwargs):
    n = len(ws)
    filtered_signal = np.zeros((n, 3))
    x = x_i.copy()
    p = p_i.copy()
    theta_as, phi_as = get_attitude_measurment(as_)
    print()
    for i in range(n):
        theta_a = theta_as[i]
        phi_a = phi_as[i]
        psi_b, phi_b, theta_b = EP2euler(x)
        if np.isnan(theta_a) or np.isnan(phi_a):
            euler = [psi_b, theta_b, phi_b]
        else:
            euler = [psi_b, theta_a, phi_a] 
        z = euler2EP(euler)  
        w_1, w_2, w_3 = ws[i]
        B = 0.5 * np.array([
            [0, -w_1, -w_2, -w_3], 
            [w_1, 0, w_3, -w_2], 
            [w_2, -w_3, 0, w_1], 
            [w_3, w_2, -w_1, 0]
        ])
        A = np.eye(4) + dt * B
        x, p = calcualte(x, p, z, A, **kwargs)
        filtered_signal[i] = EP2euler(x)
    return filtered_signal
    
    