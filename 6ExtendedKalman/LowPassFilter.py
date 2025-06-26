def filter(signal, alpha):
    m_prev = signal[0]
    rolling_avg = [m_prev]
    
    for m in signal:
        m_new = alpha * m_prev + (1 - alpha) * m
        rolling_avg.append(m_new)
        m_prev = m_new
    return rolling_avg



    