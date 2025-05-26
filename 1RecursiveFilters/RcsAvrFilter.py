def recursive_filter(z):
    global m_prev
    global n 
    
    if n not in globals():
        n = 1
    else:
        alpha = (n - 1)/n
        m = alpha * m_prev + (1-alpha) * n * z
        m_prev = m
        n += 1
        return m
    
    
    