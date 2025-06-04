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

# Kalman filter parameters

# Determined by system
# A determined each step
H = np.identity(4)
x_i = np.array([1, 0, 0, 0]) # initial state vector

# Determine by tuning
Q = np.identity(4) * 0.1
R = np.identity(4) * 0.1
p_i = np.identity(4) * 0.1  # initial covariance matrix



# For reading artificial data
# omega_1, omega_2, omega_3, t = Calibration.get_signal(length, noise_level=0.05, rng=np.random.default_rng(seed=seed), dt=dt, speed=speed, pause = 200)


# For reading real data from ArsGyro.json
df = pd.read_json('4cDynamicAltitude/Data/ArsGyro.json')
data = df["data"]

data_df = pd.DataFrame(data)

omega_1 = np.array(data_df.loc["wx", "data"])
omega_2 = np.array(data_df.loc["wy", "data"])
omega_3 = np.array(data_df.loc["wz", "data"])
t = np.arange(0, len(omega_1) * dt, dt)

# end


omegas = np.column_stack((omega_1, omega_2, omega_3))
eulers = Integrate.integrate(omegas, dt=0.01, eulers_initial=np.array([0, 0, 0]))
# eulers = AdvKalman.filter(omegas, x_i, p_i, dt=dt, eulers_initial=np.array([0, 0, 0]), H=H, Q=Q, R=R)

psi = eulers[:, 0]
theta = eulers[:, 1]
phi = eulers[:, 2]

fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6), sharex=True, sharey=True)

ax1.plot(t, omega_1, label='Signal')
ax1.set_ylabel('$\omega_1$')

ax2.plot(t, omega_2, label='Signal', color='orange')
ax2.set_ylabel('$\omega_2$')

ax3.plot(t, omega_3, label='Signal', color='green')
ax3.set_ylabel('$\omega_3$')
ax3.set_xlabel('$t$')

fig2, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(10, 6), sharex=True, sharey=True)

ax6.plot(t, psi, label='Psi', color='blue')
ax6.set_ylabel('$\psi$')
ax6.set_xlabel('$t$')
ax6.hlines(0, t.min(), t.max(), color='black', linestyle='--')

ax5.plot(t, theta, label='Theta', color='orange')
ax5.set_ylabel('$\\theta$')
ax5.hlines(0, t.min(), t.max(), color='black', linestyle='--')

ax4.plot(t, phi, label='Phi', color='green')
ax4.set_ylabel('$\phi$')
ax4.hlines(0, t.min(), t.max(), color='black', linestyle='--')

plt.show()


