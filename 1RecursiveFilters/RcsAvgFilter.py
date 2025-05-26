def calculate(z, m_prev, n):
    alpha = (n - 1) / n
    m_new = alpha * m_prev + (1 - alpha) * z
    return m_new

def filter(signal, n = 0):
    rolling_avg = []
    for z in signal:
        if n == 0:
            m_prev = z
        else:
            m_prev = calculate(z, m_prev, n)
        rolling_avg.append(m_prev)
        n += 1
    return rolling_avg