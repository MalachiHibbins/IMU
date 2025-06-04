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

def filter(signal, x_i, p_i, **kwargs):
    x = x_i
    p = p_i
    filtered_signal = []
    
    for z in signal:
        x, p = calcualte(x, p, z, **kwargs)
        filtered_signal.append(x)
    
    return np.array(filtered_signal)
    
    