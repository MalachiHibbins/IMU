def calculate(z, m_prev, n):
    # Moving average equations
    alpha = (n - 1) / n
    m_new = alpha * m_prev + (1 - alpha) * z
    print(m_new)
    return m_new

def filter(signal):
    rolling_avg = []
    n = 1
    for z in signal:
        print(n)
        if n == 1:
            m_prev = z
        else:
            m_prev = calculate(z, m_prev, n)
        rolling_avg.append(m_prev)
        n += 1
    return rolling_avg