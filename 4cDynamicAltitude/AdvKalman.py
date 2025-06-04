import numpy as np

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
    theta = np.asin2(-2*(q1*q3-q0*q2))
    phi = np.arctan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)
    return np.array([psi, theta, phi])

def euler2EP(euler_angles):
    psi, theta, phi = euler_angles
    q0 = np.cos(psi/2) * np.cos(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.sin(theta/2) * np.sin(phi/2)
    q1 = np.sin(psi/2) * np.cos(theta/2) * np.cos(phi/2) - np.cos(psi/2) * np.sin(theta/2) * np.sin(phi/2)
    q2 = np.cos(psi/2) * np.sin(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.cos(theta/2) * np.sin(phi/2)
    q3 = np.cos(psi/2) * np.cos(theta/2) * np.sin(phi/2) - np.sin(psi/2) * np.sin(theta/2) * np.cos(phi/2)
    return np.array([q0, q1, q2, q3])


def filter(signal, x_i, p_i, dt, **kwargs):
    x = x_i
    p = p_i
    filtered_signal = []
    
    for s in signal:
        z = euler2EP(s) 
        omega_1, omega_2, omega_3 = s
        
        B = 0.5 * np.array([[0, -omega_1, -omega_2, -omega_3], 
                      [omega_1, 0, omega_3, -omega_2], 
                      [omega_2, -omega_3, 0, omega_1], 
                      [omega_3, omega_2, -omega_1, 0]])
        A = np.eye(4)+dt*B
        x, p = calcualte(x, p, z, A, **kwargs)
        filtered_signal.append(EP2euler(x))
    
    return np.array(filtered_signal)
    
    