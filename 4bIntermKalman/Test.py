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
Q_s = 1.0
Q_v = 3.0
R = 20.0
s_k = 0.0  
v_k = 0.0  
P_0_s = 5.0 
P_0_v = 5.0
s_e = 2.0
v_e = 0.0

# Run Kalman filter
def run_kalman(A, H, Q_s, Q_v, R, P_0_s, P_0_v, std, dt, maximum, s_e, v_e):
    Q = np.array(([[Q_s, 0], [0, Q_v]])) 
    P_0 = np.array([[P_0_s, 0], [0, P_0_v]])  # Initial error covariance
    x_e = np.array([s_e, v_e])  # Initial state estimate
    
    # Generate signals and filter
    noisy_signal, signal = GenTestSig.get(maximum, std, rng, dt=dt)
    filtered_signal = AdvKalman.filter(noisy_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)
    
    # Extract position and velocity from filtered signal
    s_k = filtered_signal[:, 0]
    v_k = filtered_signal[:, 1]
    
    # Calculate r^2
    ss_res = np.sum((s_k - signal) ** 2)
    ss_tot = np.sum((signal - np.mean(signal)) ** 2)
    r_2 = 1 - (ss_res / ss_tot)
    
    # Calculate Root mean squared error
    rmse = np.sqrt(np.mean((s_k - signal) ** 2))
    
    # Calculate Mean Absolute Error
    mea = np.mean(np.abs(s_k - signal))
    
    return noisy_signal, signal, s_k, v_k, r_2, rmse, mea

noisy_signal, signal, s_k, v_k, r_2, rmse, mea = run_kalman(A, H, Q_s, Q_v, R, P_0_s, P_0_v, std, dt, maximum, s_e, v_e)


# Plot initial data
fig, axes = plt.subplots(2, 2, figsize=(12, 14), sharex=True)
ax1 = axes[0, 0]
ax2 = axes[1, 1]
ax3 = axes[1, 0]
ax4 = axes[0, 1]
plt.subplots_adjust(left=0.1, bottom=0.35)

# Position Plot
l1 = ax1.scatter(np.arange(len(noisy_signal)), noisy_signal, label='Noisy Signal', color='blue', alpha=0.1)
l2, = ax1.plot(signal, label='True Signal', color='green')
l3, = ax1.plot(s_k, label='Kalman Filtered Position', color='orange', alpha=0.7)
ax1.set_xlabel('Sample Index')
ax1.set_ylabel('Position')
ax1.set_title('Kalman Filtered Position')
ax1.legend()

# Velocity Plot
l4, = ax2.plot(v_k, label='Kalman Filtered Velocity', color='orange')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Velocity')
ax2.set_title('Kalman Filtered Velocity')
ax2.legend()

# Residual Plot
l5, = ax3.plot(s_k - signal, label='Velocity - True Signal', color='red', alpha=0.5)
ax3.axhline(0, color='black', linestyle='--', label='Zero Line')
ax3.set_xlabel('Sample Index')
ax3.set_ylabel('Residual')
ax3.set_title('Residuals of Kalman Filtered Position')
ax3.legend()

ax4.axis('off')
l6 = ax4.text(20, 0.5, f'$R^2$: {r_2:.4f} \n $\mu^2$: {rmse:.4f} \n $\mu$: {mea:.4f}', fontsize=20)


# Filter Faders
filter_color = 'yellow'
ax_Q_s = plt.axes([0.1, 0.25, 0.35, 0.03] )  # x and y position, width, height
ax_Q_v = plt.axes([0.1, 0.21, 0.35, 0.03] )
ax_R = plt.axes([0.1, 0.17, 0.35, 0.03] )
ax_P0_s = plt.axes([0.1, 0.13, 0.35, 0.03] )
ax_P0_v = plt.axes([0.1, 0.09, 0.35, 0.03])
ax_s_e = plt.axes([0.55, 0.13, 0.35, 0.03])
ax_v_e = plt.axes([0.55, 0.09, 0.35, 0.03])


s_Q_s = Slider(ax_Q_s, 'Q_s', 0, 10, valinit=Q_s, color = filter_color)  # Axes for slider, label, min, max, initial value
s_Q_v = Slider(ax_Q_v, 'Q_v', 0, 10, valinit=Q_v, color = filter_color)
s_R = Slider(ax_R, 'R', 0, 500, valinit=R, color = filter_color)
s_P0_s = Slider(ax_P0_s, 'P_0_s', 0, 10, valinit=P_0_s, color = filter_color)
s_P0_v = Slider(ax_P0_v, 'P_0_v', 0, 10, valinit=P_0_v, color = filter_color)
s_s_e = Slider(ax_s_e, 's_e', 0, 50, valinit=s_e, color = filter_color)
s_v_e = Slider(ax_v_e, 'v_e', 0, 10, valinit=v_e, color = filter_color)


# Graph Faders
data_colour = 'red'  
ax_max = plt.axes([0.55, 0.25, 0.35, 0.03])  # x and y position, width, height
ax_std = plt.axes([0.55, 0.21, 0.35, 0.03]) 
ax_dt = plt.axes([0.55, 0.17, 0.35, 0.03])


s_max = Slider(ax_max, 'Max Value', 0, 50, valinit=maximum, color = data_colour)
s_std = Slider(ax_std, 'std', 0, 1, valinit=std, color = data_colour)
s_dt = Slider(ax_dt, 'dt', 0.01, 0.2, valinit=dt, color = data_colour)


# Update function for sliders
def update(val):
    Q_s = s_Q_s.val
    Q_v = s_Q_v.val
    R = s_R.val
    P_0_s = s_P0_s.val
    P_0_v = s_P0_v.val
    std = s_std.val
    dt = s_dt.val
    maximum = s_max.val
    s_e = s_s_e.val
    v_e = s_v_e.val
    
    
    noisy_signal, signal, s_k, v_k, r_2, rmse, mea = run_kalman(A, H, Q_s, Q_v, R, P_0_s, P_0_v, std, dt, maximum, s_e, v_e)
    x_vals = np.arange(len(noisy_signal))
    l1.set_offsets(np.column_stack((x_vals, noisy_signal)))
    l2.set_data(x_vals, signal)
    l3.set_data(x_vals, s_k)
    l4.set_data(x_vals, v_k)
    l5.set_data(x_vals, s_k - signal)
    l6.set_text(f'$R^2$: {r_2:.4f} \n $\mu^2$: {rmse:.4f} \n $\mu$: {mea:.4f}')
    
    for ax in [ax1, ax2, ax3]:
        ax.relim()
        ax.autoscale_view()
        
    
    fig.canvas.draw_idle()
    
# Update plot each time a slider is changed
s_Q_s.on_changed(update)
s_Q_v.on_changed(update)
s_R.on_changed(update)
s_P0_s.on_changed(update)
s_P0_v.on_changed(update)
s_max.on_changed(update)
s_std.on_changed(update)
s_dt.on_changed(update)
s_s_e.on_changed(update)
s_v_e.on_changed(update)


fig.canvas.manager.set_window_title('Kalman Filter Example')
plt.show()