def GetVolt(value, rng, std=1, size = 50):
    # Generates a noisy test signal
    noise = rng.normal(loc=0, scale=std, size=size)
    noisy_signal = value + noise
    return noisy_signal 