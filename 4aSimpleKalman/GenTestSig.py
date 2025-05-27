def get(value, rng, std=1, size = 50):
    noise = rng.normal(loc=0, scale=std, size=size)
    noisy_signal = value + noise
    return noisy_signal 