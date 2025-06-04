import Calibration
import Integrate
import AdvKalman

import matplotlib.pyplot as plt
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
additional_noise_a = 1

# Kalman filter parameters

# Determined by system
# A determined each step
H = np.identity(4)
x_i = np.array([1, 0, 0, 0]) # initial state vector

# Determine by tuning
Q = np.identity(4) * 2
R = np.identity(4) * 0.1
p_i = np.identity(4) * 0.1  # initial covariance matrix


# For reading gyroscope data from ArsGyro.json
df = pd.read_json('4cDynamicAltitude/Data/ArsGyro.json')
data = df["data"]

data_df = pd.DataFrame(data)

w_1 = np.array(data_df.loc["wx", "data"])
w_2 = np.array(data_df.loc["wy", "data"])
w_3 = np.array(data_df.loc["wz", "data"])
ws = np.column_stack((w_1, w_2, w_3)) + rng.normal(0, additional_noise_w, (len(w_1), 3))
t_w = np.arange(0, len(w_1) * dt, dt)

# Calculate the euler angles using euler integration
eulers_g = Integrate.integrate(ws, dt=0.01, eulers_initial=np.array([0, 0, 0]))

psi_g = eulers_g[:, 0]
theta_g = eulers_g[:, 1]
phi_g = eulers_g[:, 2]

# For reading accelerometer data from ArsAccel.json
df = pd.read_json('4cDynamicAltitude/Data/ArsAccel.json')
data = df["data"]

data_df = pd.DataFrame(data)

a_1 = np.array(data_df.loc["fx", "data"])
a_2 = np.array(data_df.loc["fy", "data"])
a_3 = np.array(data_df.loc["fz", "data"])
as_ = np.column_stack((a_1, a_2, a_3)) + rng.normal(0, additional_noise_a, (len(a_1), 3))  
t_a = np.arange(0, len(a_1) * dt, dt)

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

# plot euler angles
alpha = 0.8
fig2, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(10, 6), sharex=True, sharey=True)

#ax4.plot(t_w, phi_g, label='$\phi_g$', color='green', alpha=alpha)
ax4.scatter(t_a, phi_a, label='$\phi_a$', color='lightgreen', alpha = alpha)
ax4.plot(t_w, phi_f, label='$\phi_f$', color='darkgreen', alpha=alpha)
ax4.set_ylabel('$\phi$')
ax4.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax4.legend()

#ax5.plot(t_w, theta_g, label='$\\theta_g$', color='orange', alpha=alpha)
ax5.scatter(t_a, theta_a, label='$\\theta_a$', color = 'yellow', alpha = alpha)
ax5.plot(t_w, theta_f, label='$\\theta_f$', color='darkorange', alpha=alpha)
ax5.set_ylabel('$\\theta$')
ax5.hlines(0, t_w.min(), t_w.max(), color='black')
ax5.legend()

#ax6.plot(t_w, psi_g, label='$\psi_g$', color='blue', alpha=alpha)
ax6.scatter(t_a, psi_a, label='$\psi_a$', color = 'lightblue', alpha=alpha)
ax6.plot(t_w, psi_f, label='$\psi_f$', color='darkblue', alpha=alpha)
ax6.set_ylabel('$\psi$')
ax6.set_xlabel('$t$')
ax6.hlines(0, t_w.min(), t_w.max(), color='black', linestyle='--')
ax6.legend()

plt.show()