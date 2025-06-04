import numpy as np

def get_signal(length, noise_level, rng, dt=1, speed=1.33, A1=1, pause = 20):
    t1 = np.arange(0, length*2*np.pi, dt)
    s1_1 = A1 * np.sin(speed * t1)
    s1_2 = np.zeros(len(t1))  
    
    t2 = np.arange(length*2*np.pi, length*2*np.pi + pause*dt, dt)
    s2_1 = np.zeros(len(t2))
    s2_2 = np.zeros(len(t2))  
    
    t3 = np.arange(length*2*np.pi + pause*dt, length*4*np.pi + pause*dt, dt)
    s3_1 = A1 * np.sin(speed * (t3 - (length*2*np.pi + pause*dt)))
    s3_2 = np.copy(s3_1) 
    
    t4 = np.arange(length*4*np.pi + pause*dt, length*4*np.pi + 2*pause*dt, dt)
    s4_1 = np.zeros(len(t4))
    s4_2 = np.zeros(len(t4))
    
    t5 = np.arange(length*4*np.pi + 2*pause*dt, length*6*np.pi + 2*pause*dt, dt)
    s5_1 = np.zeros(len(t5))  
    s5_2 = A1 * np.sin(speed * (t5 - (length*6*np.pi + pause*dt)))
    
    t = np.concatenate((t1, t2, t3, t4, t5))  
    s_1 = np.concatenate((s1_1, s2_1, s3_1, s4_1, s5_1))  
    s_2 = np.concatenate((s1_2, s2_2, s3_2, s4_2, s5_2))
    s_3 = np.zeros(len(t)) 
    
    noise_1 = rng.normal(0, noise_level, len(s_1))  # Gaussian noise
    signal_1 = s_1 + noise_1  # Combine signal and noise
    
    noise_2 = rng.normal(0, noise_level, len(s_2))  # Gaussian noise
    signal_2 = s_2 + noise_2  # Combine signal and noise
    
    noise_3 = rng.normal(0, noise_level, len(s_3))  # Gaussian noise
    signal_3 = s_3 + noise_3  # Combine signal and noise
    
    print(len(t1), len(s1_1))
    print(len(t2), len(s2_1))
    print(len(t3), len(s3_1))
    print(len(t4), len(s4_1))
    print(len(t5), len(s5_1))
    print(len(t), len(s_1))
    return signal_1, signal_2, signal_3, t  

