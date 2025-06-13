import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import OrientationKalman

g = 9.81
LSB2g = 16384**-1
LSB2w = 131**-1

class Analyse:
    def __init__ (self, data_filename, calibration_filename):
        """Converts the raw data to standard units (m/s^2 for accelerometer and rad/s for gyroscope)."""
        ax_off, ay_off, az_off, gx_off, gy_off, gz_off = self.offsets(calibration_filename)
        
        df = pd.read_csv(data_filename, header = 0)
        df.set_index("t", inplace=True)
        df = df.apply(pd.to_numeric, errors='coerce') 
        df = df.dropna()
        
        self.a = -np.column_stack([df['ax'] - ax_off, df['ay'] - ay_off, df['az'] - az_off]) * LSB2g * g
        self.w = np.column_stack([df['gx'] - gx_off, df['gy'] - gy_off, df['gz'] - gz_off ]) * LSB2w * (np.pi / 180)
        
        self.dt = np.diff(df.index).mean() / 1000
        self.theta, self.theta_a, self.theta_g = OrientationKalman.run(self.w, self.a, dt=self.dt)
        
        
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
    
    def plot_orientation_from_gyro(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        ax1.plot(self.theta_g[:, 0], label='psi (yaw)')
        ax1.set_title('Yaw (psi) from Gyroscope')
        ax2.plot(self.theta_g[:, 1], label='theta (pitch)')
        ax2.set_title('Pitch (theta) from Gyroscope')
        ax3.plot(self.theta_g[:, 2], label='phi (roll)')
        ax3.set_title('Roll (phi) from Gyroscope')
        for ax in (ax1, ax2, ax3):
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.legend()
            ax.hlines(0, 0, len(self.theta), color='black', linestyle='--', linewidth=0.5)
        fig.tight_layout()
        fig.canvas.setWindowTitle('Orientation from Gyroscope')
            
    def plot_orientation_from_accelerometer(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        ax2.plot(self.theta_a[0, :], label='theta (pitch)')
        ax2.set_title('Yaw (psi) from Accelerometer')
        ax3.plot(self.theta_a[1, :], label='phi (roll)')
        ax3.set_title('Pitch (theta) from Accelerometer')
        for ax in (ax2, ax3):
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.legend()
            ax.hlines(0, 0, len(self.theta), color='black', linestyle='--', linewidth=0.5)
        fig.tight_layout()
        fig.canvas.setWindowTitle('Orientation from Accelerometer')
        
    def plot_orientation(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        ax1.plot(self.theta[:, 0], label='psi (yaw)')
        ax1.set_title('Yaw (psi)')
        ax2.plot(self.theta[:, 1], label='theta (pitch)')
        ax2.set_title('Pitch (theta)')
        ax3.plot(self.theta[:, 2], label='phi (roll)')
        ax3.set_title('Roll (phi)')
        for ax in (ax1, ax2, ax3):
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.legend()
            ax.hlines(0, 0, len(self.theta), color='black', linestyle='--', linewidth=0.5)
        fig.tight_layout()
        fig.canvas.setWindowTitle('Orientation from Gyroscope and Accelerometer')
            
    def plot_accelaration(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        ax1.plot(self.a[:, 0], label='ax')
        ax1.set_title('Accelerometer X-axis')
        ax2.plot(self.a[:, 1], label='ay')
        ax2.set_title('Accelerometer Y-axis')
        ax3.plot(self.a[:, 2], label='az')
        ax3.set_title('Accelerometer Z-axis')
        for ax in (ax1, ax2, ax3):
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Acceleration (m/s²)')
            ax.legend()
        fig.canvas.setWindowTitle('Accelerometer Data')
        
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
        
        


test = Analyse("Data/test.csv", "Data/Calibration.csv")
test.plot_attitude()
plt.show()