import Calibration
import Integrate
import AdvKalman

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
import pandas as pd


# Data parameters
dt = 0.01
speed = 100
noise_level = 0.1
seed = 5
length = 3 # arbitary
rng = np.random.default_rng(seed)
additional_noise_w = 0
additional_noise_a = 0.5
no_signal_time = 35
max_cov_cal = int(no_signal_time/dt)

# Kalman filter parameters

# Determined by system
# A determined each step
H = np.identity(4)
x_i = np.array([1, 0, 0, 0]) # initial state vector

# Determine by tuning
q = 0.1
r = 0.1
p = 0.1
Q = np.identity(4) * q
R = np.identity(4) * r
p_i = np.identity(4) * p  # initial covariance matrix


# For reading gyroscope data from ArsGyro.json
df = pd.read_json('Data/ArsGyro.json')
data = df["data"]

data_df = pd.DataFrame(data)

w_1 = np.array(data_df.loc["wx", "data"])
w_2 = np.array(data_df.loc["wy", "data"])
w_3 = np.array(data_df.loc["wz", "data"])
ws_ = np.column_stack((w_1, w_2, w_3)) 

# For reading accelerometer data from ArsAccel.json
df = pd.read_json('Data/ArsAccel.json')
data = df["data"]

data_df = pd.DataFrame(data)

a_1 = np.array(data_df.loc["fx", "data"])
a_2 = np.array(data_df.loc["fy", "data"])
a_3 = np.array(data_df.loc["fz", "data"])
as__ = np.column_stack((a_1, a_2, a_3))

def run(Q, R, p_i, additional_noise_w, additional_noise_a):
    # Add additional noise to the gyroscope data
    ws = ws_ + rng.normal(0, additional_noise_w, (len(w_1), 3))
    t_w = np.arange(0, len(w_1) * dt, dt)

    # Calculate the euler angles using euler integration
    eulers_g = Integrate.integrate(ws, dt=0.01, eulers_initial=np.array([0, 0, 0]))

    psi_g = eulers_g[:, 0]
    theta_g = eulers_g[:, 1]
    phi_g = eulers_g[:, 2]

    # Add additional noise to the accelerometer data   
    as_ = as__ + rng.normal(0, additional_noise_a, (len(a_1), 3))
    t_a = np.arange(0, len(a_1) * dt, dt)
    
    # Calculate Q
    Q = Calibration.calculate_Q(as_, max_cov_cal)
    print(f"Calculated Q: {Q}")

    # Calculate the euler parameters from the accelerometer data
    eulers_a = AdvKalman.a2euler(as_)
    psi_a = eulers_a[0]
    theta_a = eulers_a[1]
    phi_a = eulers_a[2]

    # Use the Kalman filter to fuse the gyroscope and accelerometer data
    filtered_signal = AdvKalman.filter(as_, ws, x_i, p_i, dt, H=H, Q=Q, R=R)

    phi_f = filtered_signal[:, 0]
    theta_f = filtered_signal[:, 1]
    psi_f = filtered_signal[:, 2]
    return t_w, t_a, w_1, w_2, w_3, a_1, a_2, a_3, phi_g, theta_g, psi_g, phi_a, theta_a, psi_a, phi_f, theta_f, psi_f

t_w, t_a, w_1, w_2, w_3, a_1, a_2, a_3, phi_g, theta_g, psi_g, phi_a, theta_a, psi_a, phi_f, theta_f, psi_f = run(Q, R, p_i, additional_noise_w, additional_noise_a)

# plot w
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6), sharex=True, sharey=True)

ax1.plot(t_w, w_1, label='Signal')
ax1.set_ylabel('$\omega_1$')

ax2.plot(t_w, w_2, label='Signal', color='orange')
ax2.set_ylabel('$\omega_2$')

ax3.plot(t_w, w_3, label='Signal', color='green')
ax3.set_ylabel('$\omega_3$')
ax3.set_xlabel('$t$')


# plot a
fig2, (ax7, ax8, ax9) = plt.subplots(3, 1, figsize=(10, 6), sharex=True)

ax7.plot(t_a, a_1, label='Signal')
ax7.set_ylabel('$a_1$')

ax8.plot(t_a, a_2, label='Signal', color='orange')
ax8.set_ylabel('$a_2$')

ax9.plot(t_a, a_3, label='Signal', color='green')
ax9.set_ylabel('$a_3$')
ax9.set_xlabel('$t$')

# Plot accelarometer data as euler angles

fig5, (ax10, ax11, ax12) = plt.subplots(3, 1, figsize=(10, 6), sharex=True)

ax10.plot(t_a, phi_a, label='$\phi$', color='green')
ax10.set_ylabel('$\phi$')

ax11.plot(t_a, theta_a, label='$\\theta$', color='orange')
ax11.set_ylabel('$\\theta$')

ax12.plot(t_a, psi_a, label='$\psi$', color='blue')
ax12.set_ylabel('$\psi$')
ax12.set_xlabel('$t$')


# Plot yaw pitch roll using euler integration
fig3, (ax7, ax8, ax9) = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
alpha = 0.8
ax7.plot(t_w, phi_g, label='$\phi^-$', color='green', alpha=alpha)
ax7.plot(t_a, phi_a, label='$\phi$', color='lightgreen', linestyle='--', alpha=alpha)
ax7.set_ylabel('$\phi$')
#ax7.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax7.legend()

ax8.plot(t_w, theta_g, label='$\\theta^-$', color='orange', alpha=alpha)
ax8.plot(t_a, theta_a, label='$\\theta$', color='yellow', linestyle='--', alpha=alpha)
ax8.set_ylabel('$\\theta$')
#ax8.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax8.legend()

ax9.plot(t_w, psi_g, label='$\psi^-$', color='blue', alpha=alpha)
ax9.plot(t_a, psi_a, label='$\psi$', color='lightblue', linestyle='--', alpha=alpha)
ax9.set_ylabel('$\psi$')
ax9.set_xlabel('$t$')
#ax9.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax9.legend()

# plot euler angles
alpha = 0.8
fig4, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(12, 14), sharex=True, sharey=True)
plt.subplots_adjust(left=0.1, bottom=0.35)

l4s = ax4.scatter(t_a, phi_a, label='$\phi_a$', color='lightgreen', alpha = alpha, s = 3)
l4p, = ax4.plot(t_w, phi_f, label='$\phi_f$', color='darkgreen')
ax4.set_ylabel('$\phi$')
ax4.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax4.legend()

l5s = ax5.scatter(t_a, theta_a, label='$\\theta_a$', color = 'yellow', alpha = alpha, s = 3)
l5p, = ax5.plot(t_w, theta_f, label='$\\theta_f$', color='darkorange')
ax5.set_ylabel('$\\theta$')
ax5.hlines(0, t_w.min(), t_w.max(), color='black')
ax5.legend()

l6s = ax6.scatter(t_a, psi_a, label='$\psi_a$', color = 'lightblue', alpha=alpha, s = 3)
l6p, = ax6.plot(t_w, psi_f, label='$\psi_f$', color='darkblue')
ax6.set_ylabel('$\psi$')
ax6.set_xlabel('$t$')
ax6.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax6.legend()

# Add sliders for tuning the Kalman filter parameters
filter_color = 'yellow'
ax_q = plt.axes([0.1, 0.23, 0.8, 0.03])  # x and y position, width, height
ax_r = plt.axes([0.1, 0.19, 0.8, 0.03])
ax_p = plt.axes([0.1, 0.15, 0.8, 0.03])
ax_n_w = plt.axes([0.1, 0.11, 0.8, 0.03])
ax_n_a = plt.axes([0.1, 0.07, 0.8, 0.03])

log_q_slider = Slider(ax_q, 'log(q)', -3, 6, valinit=np.log10(Q[0, 0]), color=filter_color)  # Axes for slider, label, min, max, initial value
log_r_slider = Slider(ax_r, 'log(r)', -3, 6, valinit=np.log10(R[0, 0]), color=filter_color)
log_p_slider = Slider(ax_p, 'log(p)', -3, 6, valinit=np.log10(p_i[0, 0]), color=filter_color)
n_w_slider = Slider(ax_n_w, 'Noise w', 0, 1, valinit=additional_noise_w, color=filter_color)
n_a_slider = Slider(ax_n_a, 'Noise a', 0, 1, valinit=additional_noise_a, color=filter_color)

def update(val):
    q = 10 ** log_q_slider.val
    r = 10 ** log_r_slider.val
    p = 10 ** log_p_slider.val
    additional_noise_w = n_w_slider.val
    additional_noise_a = n_a_slider.val
    Q = np.identity(4) * q
    R = np.identity(4) * r
    p_i = np.identity(4) * p
    
    t_w, t_a, w_1, w_2, w_3, a_1, a_2, a_3, phi_g, theta_g, psi_g, phi_a, theta_a, psi_a, phi_f, theta_f, psi_f = run(Q, R, p_i, additional_noise_w, additional_noise_a)
    l4s.set_offsets(np.column_stack((t_a, phi_a)))
    l4p.set_data(t_w, phi_f)
    l5s.set_offsets(np.column_stack((t_a, theta_a)))
    l5p.set_data(t_w, theta_f)
    l6s.set_offsets(np.column_stack((t_a, psi_a)))
    l6p.set_data(t_w, psi_f)
    
    for ax in [ax4, ax5, ax6]:
        ax.relim()
        ax.autoscale_view()
    
    fig4.canvas.draw_idle()

log_q_slider.on_changed(update)
log_r_slider.on_changed(update)
log_p_slider.on_changed(update)
n_w_slider.on_changed(update)
n_a_slider.on_changed(update)

fig4.canvas.manager.set_window_title('Dynamic Altitude - Kalman Filter Tuning')
plt.show()



