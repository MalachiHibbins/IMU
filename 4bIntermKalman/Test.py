import AdvKalman
import GenTestSig

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# Fixed Gerneral Parameters
maximum = 10
std = 0.1
rng = np.random.default_rng(seed=1)
dt = 0.01

# Fixed Kalman Filter Parameters
A = np.array([[1, dt], [0, 1]]) 
H = np.array([[1, 0]])

# Variable Parameters
Q_s = 1
Q_v = 3
R = 20
s_k = 0.0  
v_k = 0.0  
P_0_s = 5.0 
P_0_v = 5.0  

def run_kalman(A, H, Q_s, Q_v, R, P_0_s, P_0_v):
    Q = np.array(([[Q_s, 0], [0, Q_v]])) 
    x_e = np.array([0, 2])  # Initial state estimate
    P_0 = np.array([[P_0_s, 0], [0, P_0_v]])  # Initial error covariance
    noisy_signal, signal = GenTestSig.get(maximum, std, rng, dt=dt)
    filtered_signal = AdvKalman.filter(noisy_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)
    s_k = filtered_signal[:, 0]
    s_v = filtered_signal[:, 1]
    return noisy_signal, signal, s_k, s_v

noisy_signal, signal, s_k, s_v = run_kalman(A, H, Q_s, Q_v, R, P_0_s, P_0_v)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
plt.subplots_adjust(left=0.1, bottom=0.35)
l1 = ax1.scatter(np.arange(len(noisy_signal)), noisy_signal, label='Noisy Signal', color='blue', alpha=0.1)
l2, = ax1.plot(signal, label='True Signal', color='green')
l3, = ax1.plot(s_k, label='Kalman Filtered Position', color='orange', alpha=0.7)
ax1.set_xlabel('Sample Index')
ax1.set_ylabel('Position')
ax1.set_title('Kalman Filter Example')
ax1.legend()

l4, = ax2.plot(s_v, label='Kalman Filtered Velocity', color='orange')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Velocity')
ax2.set_title('Kalman Filtered Velocity')
ax2.legend()


axcolor = 'lightgoldenrodyellow'
ax_Q_s = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)  # x and y position, width, height
ax_Q_v = plt.axes([0.1, 0.21, 0.8, 0.03], facecolor=axcolor)
ax_R = plt.axes([0.1, 0.17, 0.8, 0.03], facecolor=axcolor)
ax_P0_s = plt.axes([0.1, 0.13, 0.8, 0.03], facecolor=axcolor)
ax_P0_v = plt.axes([0.1, 0.09, 0.8, 0.03], facecolor=axcolor)

s_Q_s = Slider(ax_Q_s, 'Q_s', 0, 10, valinit=Q_s)  # Axes for slider, label, min, max, initial value
s_Q_v = Slider(ax_Q_v, 'Q_v', 0, 10, valinit=Q_v)
s_R = Slider(ax_R, 'R', 0, 50, valinit=R)
s_P0_s = Slider(ax_P0_s, 'P_0_s', 0, 10, valinit=P_0_s)
s_P0_v = Slider(ax_P0_v, 'P_0_v', 0, 10, valinit=P_0_v)

def update(val):
    Q_s = s_Q_s.val
    Q_v = s_Q_v.val
    R = s_R.val
    P_0_s = s_P0_s.val
    P_0_v = s_P0_v.val
    _, _, s_k, s_v = run_kalman(A, H, Q_s, Q_v, R, P_0_s, P_0_v)
    
    l3.set_ydata(s_k)
    l4.set_ydata(s_v)
    
    fig.canvas.draw_idle()
    
    
s_Q_s.on_changed(update)
s_Q_v.on_changed(update)
s_R.on_changed(update)
s_P0_s.on_changed(update)
s_P0_v.on_changed(update)
plt.show()