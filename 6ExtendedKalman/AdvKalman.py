import numpy as np

def calcualte(x, u, p, z, A, B, H, Q, R, R_u):
    # Predict state error
    x_p = A@x + B@u
    P_p = A@p@A.T + B*R_u*B.T + Q
    
    # Predict kalman gain
    K_k = P_p@H.T@np.linalg.inv(H@P_p@H.T+R)
    x_e = x_p + K_k@(z - H@x_p)
    P_e = P_p - K_k@H@P_p
    return x_e, P_e

def filter(signal, us, x_i, p_i, **kwargs):
    x = x_i
    p = p_i
    filtered_signal = []
    for z, u in zip(signal, us):
        x, p = calcualte(x, np.array([u]), p, z, **kwargs)
        filtered_signal.append(x)       
    return np.array(filtered_signal)

import matplotlib.pyplot as plt

def pure_integral(x_i, us, A, B):
    xs = []
    for i in range(len(us)):
        u_i = np.array([us[i]])
        x_n = A@x_i + B@u_i
        x_i = x_n
        xs.append(x_i)
    return np.array(xs)
    
        
        
        
        
    
    
    