import AdvKalman
import GenTestSig
import LowPassFilter

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# Gerneral Parameters
maximum = 10
std = 0.3
std_a = 0.05  # Standard deviation for acceleration noise
rng = np.random.default_rng(seed=1)
dt = 0.01
q = np.array([[(dt**4)/4, (dt**3)/2], [(dt**3)/2, dt**2]])
# Fixed Kalman Filter Parameters
A = np.array([[1, dt], [0, 1]]) 
B = np.array([[0.5 * dt**2], [dt]])
H = np.array([[1, 0]])

# Toggle lines
plot_low_pass = False
plot_model = True

alpha = 0.92



# Variable Parameters
sigma = 1
R = 20.0
R_u = 20.0
s_k = 0.0  
v_k = 0.0  
P_0_s = 5.0 
P_0_v = 5.0
s_e = 0.0
v_e = 2.4

transparency = 0.05

figsize = (10,10)

# Run Kalman filter
def run_kalman(A, B, H, sigma, q, R, R_u, P_0_s, P_0_v, std, dt, maximum, s_e, v_e):
    Q = sigma * q
    P_0 = np.array([[P_0_s, 0], [0, P_0_v]])  # Initial error covariance
    x_e = np.array([s_e, v_e])  # Initial state estimate
    
    # Generate signals and filter
    noisy_signal, signal, noisy_signal_v, signal_v, noisy_signal_a, signal_a  = GenTestSig.get(maximum, std, rng, dt=dt, noise_level_a=std_a)
    filtered_signal = AdvKalman.filter(noisy_signal, noisy_signal_a, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R, B=B, R_u=R_u)
    # Extract position and velocity from filtered signal
    s_k = filtered_signal[:, 0]
    v_k = filtered_signal[:, 1]
    
    # Generate integrated acceleration signal
    integrated_signal = AdvKalman.pure_integral(x_e, noisy_signal_a, A, B)
    s_k_i = integrated_signal[:, 0]
    v_k_i = integrated_signal[:, 1]
    
    # Calculate r^2 on position
    ss_res = np.sum((s_k - signal) ** 2)
    ss_tot = np.sum((signal - np.mean(signal)) ** 2)
    r_2 = 1 - (ss_res / ss_tot)
    
    # Calculate Root mean squared error on position
    rmse = np.sqrt(np.mean((s_k - signal) ** 2))
    
    # Calculate Mean Absolute Error on position
    mea = np.mean(np.abs(s_k - signal))
    
    # Calculate r^2 on velocity
    ss_res_v = np.sum((v_k - signal_v) ** 2)
    ss_tot_v = np.sum((signal_v - np.mean(signal_v)) ** 2)
    r_2_v = 1 - (ss_res_v / ss_tot_v)
    
    # Calculate Root mean squared error on velocity
    rmse_v = np.sqrt(np.mean((v_k - signal_v) ** 2))
    
    # Calculate Mean Absolute Error on velocity
    mea_v = np.mean(np.abs(v_k - signal_v))
    
    # differentiate s_k to get velocity
    ds_k = np.diff(s_k) / dt
    
    return noisy_signal, signal, noisy_signal_v, signal_v, noisy_signal_a, signal_a, s_k, v_k, r_2, rmse, mea, r_2_v, rmse_v, mea_v, ds_k, s_k_i, v_k_i

noisy_signal, signal, noisy_signal_v, signal_v, noisy_signal_a, signal_a, s_k, v_k, r_2, rmse, mea, r_2_v, rmse_v, mea_v, ds_k, s_k_i, v_k_i = run_kalman(A, B, H, sigma, q, R, R_u, P_0_s, P_0_v, std, dt, maximum, s_e, v_e)
x_vals = np.arange(len(noisy_signal))

signal_low_pass = LowPassFilter.filter(noisy_signal, alpha)
# Plot initial data
# fig, axes = plt.subplots(1, 2, figsize=figsize, sharex=True)
# ax1 = axes[0, 0]
# ax2 = axes[0, 1]
# #ax25 = axes[0, 2]
# ax3 = axes[1, 0]
# ax4 = axes[1, 1]
# #ax5 = axes[1, 2]
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
plt.subplots_adjust(left=0.1, bottom=0.35)

# Position Plot
l10 = ax1.scatter(np.arange(len(noisy_signal)), noisy_signal, label='Measured Position', color='blue', alpha=transparency)
l11, = ax1.plot(signal, label='True Position', color='green')
l12, = ax1.plot(s_k, label='Filtered Position', color='orange', linestyle='dashed')
if plot_low_pass:
    ax1.plot(signal_low_pass, label='Low Pass Filtered Position', color='red', linestyle='dotted')
if plot_model:
    ax1.plot(s_k_i, label='Double Integrated Accelarometer Data', color='purple')
ax1.set_ylabel('Position')
ax1.set_title('Kalman Filtered Position')
ax1.legend(loc="upper right")    
    
    
# Velocity Plot
l20, = ax2.plot(signal_v, label='True Velocity', color='green')
l21, = ax2.plot(v_k, label='Filtered Velocity', color='orange', linestyle='dashed')
if plot_model:
    l44, = ax2.plot(v_k_i, label='Integrated Accelarometer Data', color='purple')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Velocity')
ax2.set_title('Kalman Filtered Velocity')
ax2.legend(loc="upper right")


# Display R^2, RMSE, and MAE
l13 = ax1.text(40, -1, f'$r^2$: {r_2:.4f} \n MSE: {rmse:.4f} \n MAE: {mea:.4f}', fontsize=10)
l22 = ax2.text(750, -1.7, f'$r^2$: {r_2_v:.4f} \n MSE: {rmse_v:.4f} \n MAE: {mea_v:.4f}', fontsize=10)

# Filter sliders
filter_color = 'yellow'
width = 0.8
height = 0.03
ax_sigma = plt.axes([0.1, 0.25, width, height])  # x and y position, width, height
ax_R = plt.axes([0.1, 0.21, width, height])
ax_P0_s = plt.axes([0.1, 0.17, width, height])
ax_P0_v = plt.axes([0.1, 0.13, width, height])
ax_s_e = plt.axes([0.1, 0.09, width, height])
ax_v_e = plt.axes([0.1, 0.05, width, height])


log_sigma = Slider(ax_sigma, '$\log(\sigma_a)$', -3, 10, valinit=np.log10(sigma), color = filter_color)  # Axes for slider, label, min, max, initial value
log_s_R = Slider(ax_R, '$\log(\sigma_s)$', -3, 6, valinit=np.log10(R), color = filter_color)
s_P0_s = Slider(ax_P0_s, '$P_0^s$', 0, 10, valinit=P_0_s, color = filter_color)
s_P0_v = Slider(ax_P0_v, '$P_0^v$', 0, 10, valinit=P_0_v, color = filter_color)
s_s_e = Slider(ax_s_e, '$s_0$', 0, 50, valinit=s_e, color = filter_color)
s_v_e = Slider(ax_v_e, '$v_0$', 0, 10, valinit=v_e, color = filter_color)


# Update function for sliders
def update(val):
    sigma = 10**log_sigma.val
    R = 10**log_s_R.val
    P_0_s = s_P0_s.val
    P_0_v = s_P0_v.val
    # std = s_std.val
    # dt = 10**s_log_dt.val
    # maximum = s_max.val
    s_e = s_s_e.val
    v_e = s_v_e.val
    
    
    noisy_signal, signal, noisy_signal_v, signal_v, noisy_signal_a, signal_a, s_k, v_k, r_2, rmse, mea, r_2_v, rmse_v, mea_v, ds_k, s_k_i, v_k_i = run_kalman(A, B, H, sigma, q, R, R_u, P_0_s, P_0_v, std, dt, maximum, s_e, v_e)
    l10.set_offsets(np.column_stack((x_vals, noisy_signal)))
    l11.set_data(x_vals, signal)
    l12.set_data(x_vals, s_k)
    l21.set_data(x_vals, v_k)
    l20.set_data(x_vals, signal_v)
    l13.set_text(f'$r^2$: {r_2:.4f} \n MSE: {rmse:.4f} \n MAE: {mea:.4f}')
    l22.set_text(f'$r^2$: {r_2_v:.4f} \n MSE: {rmse_v:.4f} \n MAE: {mea_v:.4f}')
    
    for ax in [ax1, ax2]:
        ax.relim()
        ax.autoscale_view()
        
    
    fig.canvas.draw_idle()
    
# Update plot each time a slider is changed
log_sigma.on_changed(update)
log_s_R.on_changed(update)
s_P0_s.on_changed(update)
s_P0_v.on_changed(update)
s_s_e.on_changed(update)
s_v_e.on_changed(update)


fig.canvas.manager.set_window_title('Kalman Filter Example')
plt.show()