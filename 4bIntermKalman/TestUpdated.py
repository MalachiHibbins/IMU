import AdvKalman
import GenTestSig

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# Gerneral Parameters
maximum = 10
std = 0.1
rng = np.random.default_rng(seed=1)
dt = 0.01
q = np.array([[(dt**4)/4, (dt**3)/2], [(dt**3)/2, dt**2]])
# Fixed Kalman Filter Parameters
A = np.array([[1, dt], [0, 1]]) 
H = np.array([[1, 0]])
plot_residuels = False # These can be changed to add residuals graphs underneath the main graphs
plot_differentiated_filtered = False
alpha_scatter = 0.1


# Variable Parameters
sigma = 1
R = 20.0
s_k = 0.0  
v_k = 0.0  
P_0_s = 5.0 
P_0_v = 5.0
s_e = 0.0
v_e = 2.0

figsize = (10,10)

# Run Kalman filter
def run_kalman(A, H, sigma, q, R, P_0_s, P_0_v, std, dt, maximum, s_e, v_e):
    Q = sigma * q
    P_0 = np.array([[P_0_s, 0], [0, P_0_v]])  # Initial error covariance
    x_e = np.array([s_e, v_e])  # Initial state estimate
    
    # Generate signals and filter
    noisy_signal, signal, noisy_signal_v, signal_v = GenTestSig.get(maximum, std, rng, dt=dt)
    filtered_signal = AdvKalman.filter(noisy_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)
    
    # Extract position and velocity from filtered signal
    s_k = filtered_signal[:, 0]
    v_k = filtered_signal[:, 1]
    
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
    
    return noisy_signal, signal, noisy_signal_v, signal_v, s_k, v_k, r_2, rmse, mea, r_2_v, rmse_v, mea_v, ds_k

noisy_signal, signal, noisy_signal_v, signal_v, s_k, v_k, r_2, rmse, mea, r_2_v, rmse_v, mea_v, ds_k = run_kalman(A, H, sigma, q, R, P_0_s, P_0_v, std, dt, maximum, s_e, v_e)
x_vals = np.arange(len(noisy_signal))

# Plot initial data
if plot_residuels:
    fig, axes = plt.subplots(2, 2, figsize=figsize, sharex=True)
    ax1 = axes[0, 0]
    ax2 = axes[0, 1]
    ax3 = axes[1, 0]
    ax4 = axes[1, 1]
else:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
plt.subplots_adjust(left=0.1, bottom=0.35)



# Position Plot
l1 = ax1.scatter(np.arange(len(noisy_signal)), noisy_signal, label='Measured Position', color='blue', alpha=alpha_scatter)
l11, = ax1.plot(signal, label='True Position', color='green')
l12, = ax1.plot(s_k, label='Filtered Position', color='orange', alpha=0.7)
ax1.set_xlabel('Sample Index')
ax1.set_ylabel('Position')
ax1.set_title('Kalman Filtered Position')
ax1.legend(loc="upper right")



# Velocity Plot
if plot_differentiated_filtered:
    l22 = ax2.scatter(x_vals[:-1], ds_k, label='Differentiated Filtered Position', color='red', alpha=alpha_scatter)
#l41 = ax2.scatter(np.arange(len(noisy_signal)), noisy_signal_v, label='Noisy Velocity', color='blue', alpha=0.1)
l21, = ax2.plot(signal_v, label='True Velocity', color='green')
l20, = ax2.plot(v_k, label='Filtered Velocity', color='orange')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Velocity')
ax2.set_title('Kalman Filtered Velocity')
ax2.legend(loc="upper right")

if plot_residuels:
    # Residual Plot position
    l30, = ax3.plot(s_k - signal, label='Filtred Position -  True Position', color='red', alpha=0.5)
    ax3.axhline(0, color='black', linestyle='--', label='Zero Line')
    ax3.set_xlabel('Sample Index')
    ax3.set_ylabel('Residual')
    ax3.set_title('Residuals of Kalman Filtered Position')
    ax3.legend(loc="upper right")

    # Residual Plot velocity
    l40, = ax4.plot(v_k - signal_v, label='Filtered Velocity - True Velocity', color='red', alpha=0.5)
    ax4.axhline(0, color='black', linestyle='--', label='Zero Line')
    ax4.set_xlabel('Sample Index')
    ax4.set_ylabel('Residual')
    ax4.set_title('Residuals of Kalman Filtered Velocity')
    ax4.legend(loc="upper right")

# Display R^2, RMSE, and MAE
t1 = ax1.text(40, -1, f'$r^2$: {r_2:.4f} \n MSE: {rmse:.4f} \n MAE: {mea:.4f}', fontsize=10)
t2 = ax2.text(750, -1.7, f'$r^2$: {r_2_v:.4f} \n MSE: {rmse_v:.4f} \n MAE: {mea_v:.4f}', fontsize=10)

# Filter sliders
filter_color = 'yellow'
width = 0.8
height = 0.03
ax_sigma = plt.axes([0.1, 0.25, width, height] )  # x and y position, width, height
ax_R = plt.axes([0.1, 0.21, width, height] )
ax_P0_s = plt.axes([0.1, 0.17, width, height] )
ax_P0_v = plt.axes([0.1, 0.13, width, height])
ax_s_e = plt.axes([0.1, 0.09, width, height])
ax_v_e = plt.axes([0.1, 0.05, width, height])


log_sigma = Slider(ax_sigma, '$\log(\sigma_Q)$', -3, 10, valinit=np.log10(sigma), color = filter_color)  # Axes for slider, label, min, max, initial value
log_s_R = Slider(ax_R, '$\log(\sigma_R)$', -3, 6, valinit=np.log10(R), color = filter_color)
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
    
    noisy_signal, signal, noisy_signal_v, signal_v, s_k, v_k, r_2, rmse, mea, r_2_v, rmse_v, mea_v, ds_k = run_kalman(A, H, sigma, q, R, P_0_s, P_0_v, std, dt, maximum, s_e, v_e)
    l1.set_offsets(np.column_stack((x_vals, noisy_signal)))
    l11.set_data(x_vals, signal)
    l12.set_data(x_vals, s_k)
    l20.set_data(x_vals, v_k)
    #l41.set_offsets(np.column_stack((x_vals, noisy_signal_v)))
    l21.set_data(x_vals, signal_v)
    if plot_differentiated_filtered:
        l22.set_offsets(np.column_stack((x_vals[:-1], ds_k)))
    if plot_residuels:
        l30.set_data(x_vals, s_k - signal)
        l40.set_data(x_vals, v_k - signal_v)
    t1.set_text(f'$r^2$: {r_2:.4f} \n MSE: {rmse:.4f} \n MAE: {mea:.4f}')
    t2.set_text(f'$r^2$: {r_2_v:.4f} \n MSE: {rmse_v:.4f} \n MAE: {mea_v:.4f}')
    
    
    for ax in [ax1, ax2, ax3, ax4]:
        ax.relim()
        ax.autoscale_view()
        
    
    fig.canvas.draw_idle()
    
# Update plot each time a slider is changed
log_sigma.on_changed(update)
log_s_R.on_changed(update)
s_P0_s.on_changed(update)
s_P0_v.on_changed(update)
# s_max.on_changed(update)
# s_std.on_changed(update)
# s_log_dt.on_changed(update)
s_s_e.on_changed(update)
s_v_e.on_changed(update)


fig.canvas.manager.set_window_title('Kalman Filter Example')
plt.show()