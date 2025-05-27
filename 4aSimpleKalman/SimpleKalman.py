def calcualte(x, p, z, A = 1, H = 1, Q = 0, R = 4):
    # Predict state error
    x_p = A*x
    P_p = A*p*A + Q
    
    # Predict kalman gain
    K_k = P_p*H*(H*P_p*H+R)**-1
    x_e = x_p + K_k*(z - H*x_p)
    P_e = (1 - K_k*H)*P_p
    return x_e, P_e

def filter(signal, x_i = 5, p_i = 0, **kwargs):
    x = x_i
    p = p_i
    filtered_signal = []
    
    for z in signal:
        x, p = calcualte(x, p, z, **kwargs)
        filtered_signal.append(x)
    
    return filtered_signal
    
    