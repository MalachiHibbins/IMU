import numpy as np

def filter_signal(signal, window, moving_averages=[]):
    k = len(moving_averages) + window
    
    if moving_averages == []:
        average_new = signal[:window].mean()
        
    elif k < len(signal):
        average_prev = moving_averages[-1]
        average_new = average_prev + (signal[k] - signal[k - window]) / window
    
    else:
        return np.arange(window, k, 1), moving_averages
    
    moving_averages.append(average_new)
    return filter_signal(signal, window, moving_averages)