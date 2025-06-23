import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import scipy.constants as sc

import OrientationKalman

g = sc.g  # Acceleration due to gravity in m/s^2
LSB2g = 16384**-1
LSB2w = 131**-1

class Analyse:
    def __init__ (self, data_filename, calibration_filename):
        """Converts the raw data to standard units (m/s^2 for accelerometer and rad/s for gyroscope)."""
        ax_off, ay_off, az_off, gx_off, gy_off, gz_off = self.offsets(calibration_filename)
        self.q = 10**-1.6  # Process noise covariance
        self.r = 10**0.7  # Measurement noise covariance
        
        df = pd.read_csv(data_filename, header = 0)
        df.set_index("t", inplace=True)
        df = df.apply(pd.to_numeric, errors='coerce') 
        df = df.dropna()
        
        self.a = -np.column_stack([df['ax'] - ax_off, df['ay'] - ay_off, df['az'] - az_off]) * LSB2g * g
        self.w = np.column_stack([df['gx'] - gx_off, df['gy'] - gy_off, df['gz'] - gz_off ]) * LSB2w * (np.pi / 180)
        
        self.dt = np.diff(df.index).mean() / 1000
        self.t = df.index.values / 1000  # Convert to seconds
        self.theta, self.theta_a, self.theta_g = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r)
        
    def setup_sliders(self, fig, length = 0.8, height = 0.03):
        axq = fig.add_axes([0.1, 0.01, length, height], facecolor='lightgoldenrodyellow')
        axr = fig.add_axes([0.1, 0.05, length, height], facecolor='lightgoldenrodyellow')
        
        self.s_logq = Slider(axq, '$\log(q)$', -3, 5, valinit=np.log10(self.q), valstep=0.1)
        self.s_logr = Slider(axr, '$\log(r)$', -3, 5, valinit=np.log10(self.r), valstep=0.1)
        
    def check_slider_updates(self):
        self.s_logq.on_changed(lambda val: self.update())
        self.s_logr.on_changed(lambda val: self.update())
        
    def update(self):
        self.q = 10**self.s_logq.val
        self.r = 10**self.s_logr.val
        self.theta, self.theta_a, self.theta_g = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r)
        self.line_yaw.set_ydata(self.theta[:, 0])
        self.line_pitch.set_ydata(self.theta[:, 1])
        self.line_roll.set_ydata(self.theta[:, 2])
        fig.canvas.draw_idle()
        
    def offsets(self, filename):
        df = pd.read_csv(filename, header = 0)
        df.set_index("t", inplace=True)
        df = df.apply(pd.to_numeric, errors='coerce') 
        df = df.dropna()
        ax_offset = df['ax'].mean()
        ay_offset = df['ay'].mean()
        az_offset = df['az'].mean() - (1/LSB2g)
        wx_offset = df['gx'].mean()
        wy_offset = df['gy'].mean()
        wz_offset = df['gz'].mean()
        return ax_offset, ay_offset, az_offset, wx_offset, wy_offset, wz_offset
        
        
    def plot_raw_data(self):
        """Plots the raw data of the IMU."""
        fig, axs = plt.subplots(3, 2, figsize=(12, 8))
        axs[0, 0].plot(self.a[:, 0], label='ax')
        axs[0, 0].set_title('Accelerometer X-axis')
        axs[1, 0].plot(self.a[:, 1], label='ay')
        axs[1, 0].set_title('Accelerometer Y-axis')
        axs[2, 0].plot(self.a[:, 2], label='az')
        axs[2, 0].set_title('Accelerometer Z-axis')
        axs[0, 1].plot(self.w[:, 0], label='gx')
        axs[0, 1].set_title('Gyroscope X-axis')
        axs[1, 1].plot(self.w[:, 1], label='gy')
        axs[1, 1].set_title('Gyroscope Y-axis')
        axs[2, 1].plot(self.w[:, 2], label='gz')
        axs[2, 1].set_title('Gyroscope Z-axis')

        for ax in axs.flat:
            ax.legend()
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')

        fig.tight_layout()
        fig.canvas.setWindowTitle('Raw IMU Data')
        
    def setup_plot(self):
        length = len(self.theta_g[:, 0])
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].set_ylabel('Yaw, $\psi$ (rad)')
        axs[0].hlines(y=0, xmin=0, xmax=length, colors = 'k', linestyle = 'dashed')
        axs[1].set_ylabel('Pitch, $\\theta$ (rad)')
        axs[1].hlines(y=0, xmin=0, xmax=length, colors = 'k', linestyle = 'dashed')
        axs[2].set_ylabel('Roll, $\phi$ (rad)')
        axs[2].hlines(y=0, xmin=0, xmax=length, colors = 'k', linestyle = 'dashed')
        axs[2].set_xlabel('time (s)')
        return fig, axs
    
    def plot_orientation_from_gyro(self, fig = None, axs = None, alpha = 0.8):
        if fig == None and axs == None:
            fig, axs = self.setup_plot()
        axs[0].plot(self.theta_g[:, 0], label='gyro', alpha = alpha)
        axs[1].plot(self.theta_g[:, 1], label='gyro', alpha = alpha)
        axs[2].plot(self.theta_g[:, 2], label='gyro', alpha = alpha)
        return fig, axs
            
    def plot_orientation_from_accelerometer(self, fig = None, axs = None, alpha = 0.8):
        if fig == None and axs == None:
            fig, axs = self.setup_plot()
        axs[1].plot(self.theta_a[0, :], label='accelarometer', alpha = alpha)
        axs[2].plot(self.theta_a[1, :], label='accelarometer', alpha = alpha)
        return fig, axs
        
    def plot_orientation_fused(self, fig = None, axs = None, alpha = 0.8):
        if fig == None and axs == None:
            fig, axs = self.setup_plot()
            plt.subplots_adjust(left=0.1, bottom=0.15)
        
        self.setup_sliders(fig)
        self.line_yaw, = axs[0].plot(self.theta[:, 0], label='Kalman', alpha = alpha)
        self.line_pitch, = axs[1].plot(self.theta[:, 1], label='Kalman', alpha = alpha)
        self.line_roll, = axs[2].plot(self.theta[:, 2], label='Kalman', alpha = alpha)
        axs[2].set_xlabel('time (s)')
        self.check_slider_updates()
        return fig, axs
        

            
    def plot_accelaration(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        ax1.plot(self.a[:, 0], label='ax')
        ax1.ylabel('X Acceleration (ms$^{-2}$)')
        ax2.plot(self.a[:, 1], label='ay')
        ax2.ylabel('Y Acceleration (ms$^{-2}$)')
        ax3.plot(self.a[:, 2], label='az')
        ax3.ylabel('Z Acceleration (ms$^{-2}$)')
        ax3.set_ylabel('Acceleration (m/s²)')
        ax3.set_xlabel('Time')
        
    def plot_attitude(self):
        """Plots a 3D animation of the orientation, showing all three IMU axes."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        quivers = []

        for i in range(len(self.theta)):
            psi, theta, phi = self.theta[i]  # yaw, pitch, roll

            # Rotation matrices (3-2-1: yaw-pitch-roll)
            Rz = np.array([
                [np.cos(psi), -np.sin(psi), 0],
                [np.sin(psi),  np.cos(psi), 0],
                [0,            0,           1]
            ])
            Ry = np.array([
                [np.cos(theta), 0, np.sin(theta)],
                [0,             1, 0],
                [-np.sin(theta),0, np.cos(theta)]
            ])
            Rx = np.array([
                [1, 0,            0],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi),  np.cos(phi)]
            ])
            R = Rz @ Ry @ Rx

            # Rotated axes
            x_axis = R @ np.array([1, 0, 0])
            y_axis = R @ np.array([0, 1, 0])
            z_axis = R @ np.array([0, 0, 1])

            # Remove previous arrows
            for q in quivers:
                q.remove()
            quivers = []

            # Plot new arrows for each axis
            quivers.append(ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='r', length=0.8, normalize=True, label='X'))
            quivers.append(ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='g', length=1.2, normalize=True, label='Y'))
            quivers.append(ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='b', length=0.8, normalize=True, label='Z'))

            plt.pause(0.02)
        
        


test = Analyse("Data/angle12.csv", "Data/cal12.csv")
test.plot_raw_data()
#fig, axs = test.plot_orientation_fused()
fig, axs = test.plot_orientation_from_accelerometer()
fig, axs = test.plot_orientation_from_gyro(fig, axs)
ax1, ax2, ax3 = axs
ax1.legend()
ax2.legend()
ax3.legend()
plt.show()