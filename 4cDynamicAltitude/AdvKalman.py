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
    psi = np.arctan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)
    theta = np.arcsin(-2*(q1*q3-q0*q2))
    phi = np.arctan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)
    return np.array([psi, theta, phi])

def euler2EP(euler_angles):
    psi, theta, phi = euler_angles
    q0 = np.cos(psi/2) * np.cos(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.sin(theta/2) * np.sin(phi/2)
    q1 = np.sin(psi/2) * np.cos(theta/2) * np.cos(phi/2) - np.cos(psi/2) * np.sin(theta/2) * np.sin(phi/2)
    q2 = np.cos(psi/2) * np.sin(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.cos(theta/2) * np.sin(phi/2)
    q3 = np.cos(psi/2) * np.cos(theta/2) * np.sin(phi/2) - np.sin(psi/2) * np.sin(theta/2) * np.cos(phi/2)
    return np.array([q0, q1, q2, q3])

def a2euler(as_):
    thetas = []
    phis = []
    psis = []
    for a in as_:
        a1, a2, a3 = a
        theta = np.arcsin(a1/sc.g)
        phi = -np.arcsin(a2/(sc.g*np.cos(theta)))
        psi = 0
        thetas.append(theta)
        phis.append(phi)
        psis.append(psi)
    return np.array([psis, thetas, phis])
        
def filter(as_, ws, x_i, p_i, dt, **kwargs):
    x = x_i
    p = p_i
    zs = euler2EP(a2euler(as_)).T
    filtered_signal = []
    
    for w, z in zip(ws, zs):
        w_1, w_2, w_3 = w
        B = 0.5 * np.array([[0, -w_1, -w_2, -w_3], 
                      [w_1, 0, w_3, -w_2], 
                      [w_2, -w_3, 0, w_1], 
                      [w_3, w_2, -w_1, 0]])
        A = np.eye(4)+dt*B
        x, p = calcualte(x, p, z, A, **kwargs)
        filtered_signal.append(EP2euler(x))
    
    return np.array(filtered_signal)
    
    