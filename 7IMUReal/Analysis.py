import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import scipy.constants as sc

import OrientationKalman

g = sc.g  # Acceleration due to gravity in m/s^2

# used for concersion between MPU6050 units for velocity and position
LSB2g = 16384**-1
LSB2w = 131**-1

class AnalyseMPU: 
    """Class to read and analyse data from the MPU6050 sensor.

    """
    def __init__ (self, folder_name, q = 10**-1.6, r = 10**-1.6):
        """
        - Reads in data from a csv file into a pandas dataframe
        - converts the data to standard units (m/s^2 for accelerometer and rad/s for gyroscope)
        - removes background noise by subtracting the offset calculated from data in the calibration file
        - stores this data as attributes of the object. 
        - calculates the time step from the data
    
        Args:
            folder_name (str): Name of the folder containing the data files.
            q (float): Process noise covariance for the Kalman filter. Default is 10**-1.6.
            r (float): Measurement noise covariance for the Kalman filter. Default is 10**-1.6.
        
        Returns:
            None
        """
        ax_off, ay_off, az_off, gx_off, gy_off, gz_off = self._background(f"MPUData/{folder_name}/calibration.csv")
        self.q = q  # Process noise covariance
        self.r = r  # Measurement noise covariance
        
        
        # Reads in file
        df = pd.read_csv(f"MPUData/{folder_name}/data.csv", header = 0)
        df.set_index("t", inplace=True)
        df = df.apply(pd.to_numeric, errors='coerce') 
        df = df.dropna()
        
        # Saved data as attributes of the object
        self.a = -np.column_stack([df['ax'] - ax_off, df['ay'] - ay_off, df['az'] - az_off]) * LSB2g * g # Convert to m/s^2
        self.w = np.column_stack([df['gx'] - gx_off, df['gy'] - gy_off, df['gz'] - gz_off ]) * LSB2w * (np.pi / 180) # Convert to radians/s
        
        self.dt = np.diff(df.index).mean() / 1000 # Convert to seconds
        self.t = df.index.values / 1000  # Convert to seconds
        # Kalman filtered attitude, acclearometer attitude, gyroscope attitude
        self.theta, self.theta_a, self.theta_g, self.zs = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r)
        
    def _setup_sliders(self, fig, length = 0.725, height = 0.03):
        """
        - Creates sliders for adjusting q and r values in the Kalman filter, Allowing for real time adjustment.
        - Adds the sliders to the figure.
        
        Args:
            fig (matplotlib.figure.Figure): The figure to which the sliders will be added.
            length (float): Length of the sliders. Default is 0.725.
            height (float): Height of the sliders. Default is 0.03.
        Returns:
            None
        """
        # Creates sliders for graphs
        # parameters [x position y position length height], facecolour
        axq = fig.add_axes([0.125, 0.00, length, height], facecolor='lightgoldenrodyellow')
        axr = fig.add_axes([0.125, 0.02, length, height], facecolor='lightgoldenrodyellow')
        
        # parameters [acis, label, min, max, initial value, step size]
        self.s_logq = Slider(axq, '$\log(q)$', -3, 5, valinit=np.log10(self.q), valstep=0.1)
        self.s_logr = Slider(axr, '$\log(r)$', -3, 5, valinit=np.log10(self.r), valstep=0.1)
        
    def _update(self):
        """
        - Updates the kalman filter based on the new values in the sliders.
        Args:
            None
        Returns:
            None
        """
        # Handles the sliders logarithmic effects
        self.q = 10**self.s_logq.val
        self.r = 10**self.s_logr.val
        
        # Recalculates the Kalman filter with the new q and r values only self.theta will be updated
        self.theta, self.theta_a, self.theta_g, self.zs = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r)
        
        # Updates graph data so graph can be redrawn
        self.line_yaw.set_offsets(np.column_stack((self.t, self.theta[:, 0])))
        self.line_pitch.set_offsets(np.column_stack((self.t, self.theta[:, 1])))
        self.line_roll.set_offsets(np.column_stack((self.t, self.theta[:, 2])))
        fig.canvas.draw_idle()
        
    def _background(self, filename):
        """"
        - Calculates the offsets for the accelerometer and gyroscope data.
        - zeroes the measurments
        Args:
            filename (str): The name of the file with in the specified folder containing calibration data.
        Returns:
            offsets (tuple): A tuple containing the offsets for ax, ay, az, gx, gy, gz.
        """
        df = pd.read_csv(filename, header = 0)
        df.set_index("t", inplace=True)
        df = df.apply(pd.to_numeric, errors='coerce') 
        df = df.dropna()
        
        # Calcualtes offset based on
        ax_offset = df['ax'].mean()
        ay_offset = df['ay'].mean()
        az_offset = df['az'].mean() - (1/LSB2g)
        wx_offset = df['gx'].mean()
        wy_offset = df['gy'].mean()
        wz_offset = df['gz'].mean()
        return ax_offset, ay_offset, az_offset, wx_offset, wy_offset, wz_offset
        
        
    def plot_raw_data(self):
        """
        - Plots the data directly from the sensors
        - Creates a 3x2 grid of subplots for accelerometer and gyroscope data.
        Args:
            None
        Returns:
            None
        """
        fig, axs = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
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
        
    def _setup_plot(self):
        """
        - Adds cuttoff lines to plot and creates a figure and axes for plotting.
        - Sets up a 3x1 grid of subplots for yaw, pitch, and roll.
        Args:
            None
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots.
        """
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].set_ylabel('Yaw, $\psi$ (rad)')
        axs[0].hlines(y=-np.pi, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        axs[0].hlines(y=np.pi, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        axs[1].set_ylabel('Pitch, $\\theta$ (rad)')
        axs[1].hlines(y=0, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        axs[1].hlines(y=np.pi/2, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        axs[1].hlines(y=-np.pi/2, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        axs[2].set_ylabel('Roll, $\phi$ (rad)')
        axs[2].hlines(y=-np.pi, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        axs[2].hlines(y=np.pi, xmin=0, xmax=max(self.t), colors = 'k', linestyle = 'dashed')
        return fig, axs
    
    def plot_orientation_from_gyro(self, fig = None, axs = None, alpha = 0.8):
        """
        - Plots the orientation from gyroscope data alone i.e. using integration
        Args:
            fig (matplotlib.figure.Figure, optional): The figure to plot on. If None, a new figure will be created.
            axs (numpy.ndarray, optional): The axes to plot on. If None, new axes will be created.
            alpha (float, optional): Transparency of the points in the scatter plot. Default is 0.8.
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots.
        """
        if fig == None and axs == None:
            fig, axs = self._setup_plot()
        index = np.arange(len(self.theta_g[:, 0]))
        axs[0].scatter(self.t, self.theta_g[:, 0], label='Gyro (only)', alpha = alpha, color='orange', marker='x', s = 2)
        axs[1].scatter(self.t, self.theta_g[:, 1], label='Gyro (only)', alpha = alpha, color='orange', marker='x', s = 2)
        axs[2].scatter(self.t, self.theta_g[:, 2], label='Gyro (only)', alpha = alpha, color='orange', marker='x', s = 2)
        axs[2].set_xlabel('Time (s)')
        return fig, axs
            
    def plot_orientation_from_accelerometer(self, fig = None, axs = None, alpha = 0.8):
        """
        - Plots the orientation from accelerometer data alone
        - Only plots on the yaw and pitch axes, the accelerometer cannot measure roll.
        Args:
            fig (matplotlib.figure.Figure, optional): The figure to plot on. If None, a new figure will be created.
            axs (numpy.ndarray, optional): The axes to plot on. If None, new axes will be created.
            alpha (float, optional): Transparency of the points in the scatter plot. Default is 0.8.
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots.
        """
        if fig == None and axs == None:
            fig, axs = self._setup_plot()
        axs[1].scatter(self.t, self.theta_a[0, :], label='accelarometer', alpha = alpha, color='green', marker='x', s = 2)
        axs[2].scatter(self.t, self.theta_a[1, :], label='accelarometer', alpha = alpha, color='green', marker='x', s = 2)
        return fig, axs
        
    def plot_orientation_fused(self, fig = None, axs = None, alpha = 0.8):
        """
        - Plots the orientation from the Kalman filter
        Args:
            fig (matplotlib.figure.Figure, optional): The figure to plot on. If None, a new figure will be created.
            axs (numpy.ndarray, optional): The axes to plot on. If None, new axes will be created.
            alpha (float, optional): Transparency of the points in the scatter plot. Default is 0.8.
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots.
        """
        if fig == None and axs == None:
            fig, axs = self._setup_plot()
            # Adds space to the figure so sliders can be added at the bottom
            fig.subplots_adjust(left=0.1, bottom=0.4)
        
        self._setup_sliders(fig)
        self.line_yaw = axs[0].scatter(self.t, self.theta[:, 0], label='Kalman (Fused)', alpha = alpha, color='blue', marker='x', s = 2)
        self.line_pitch = axs[1].scatter(self.t, self.theta[:, 1], label='Kalman (Fused)', alpha = alpha, color='blue', marker='x', s = 2)
        self.line_roll = axs[2].scatter(self.t, self.theta[:, 2], label='Kalman (Fused)', alpha = alpha, color='blue', marker='x', s = 2)
        self.s_logq.on_changed(lambda val: self._update())
        self.s_logr.on_changed(lambda val: self._update())
        return fig, axs
    
    def plot_measurment(self, fig = None, axs = None, alpha = 0.8):
        """
        Plots the orientation measurment used to update the model from sensors.
        Args:
            fig (matplotlib.figure.Figure, optional): The figure to plot on. If None, a new figure will be created.
            axs (numpy.ndarray, optional): The axes to plot on. If None, new axes will be created.
            alpha (float, optional): Transparency of the points in the scatter plot. Default is 0.8.
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots.
        """
        if fig == None and axs == None:
            fig, axs = self._setup_plot()
        axs[0].scatter(self.t, self.zs[:, 0], label='Acc (only)', alpha = alpha, color='red', marker='x', s = 2)
        axs[1].scatter(self.t, self.zs[:, 1], label='Acc (only)', alpha = alpha, color='red', marker='x', s = 2)
        axs[2].scatter(self.t, self.zs[:, 2], label='Acc (only)', alpha = alpha, color='red', marker='x', s = 2)
        return fig, axs
        
        
    def plot_attitude(self, data = "kalman", pause = 0.02):
        """
        Plots a 3D animation of the orientation.
        Args:
            data (str): The type of data to plot. Can be "kalman" or "gyro".
            pause (float): Pause duration between frames in seconds. Default is 0.02.
        Returns:
            None
        """
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
            if data == "kalman":
                psi, theta, phi = self.theta[i] 
            elif data == "gyro":
                psi, theta, phi = self.theta_g[i]

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

            plt.pause(pause)
        
class AnalysePhone(AnalyseMPU): #
    """
    - Reads and analyses data from the sensorlogger app on a phone.
    - Inherits from AnalyseMPU
    """
    def __init__(self, FolderName, q = 10**-1.6, r = 10**-1.6):
        """
        - Reads Gravity, Gyroscope, Magnetometer and Orientation data from a files
        - Puts this data into a single pandas dataframe
        - saves the data as attributes of the object.
        Args:
            FolderName (str): Name of the folder containing the sensor data files.
            q (float): Process noise covariance for the Kalman filter. Default is 10**-1.6.
            r (float): Measurement noise covariance for the Kalman filter. Default is 10**-1.6.
        Returns:
            None
        """
        self.q = q
        self.r = r  
        
        # get accelarometer data in x,y,z directions
        df_a = pd.read_csv("SensorLoggerData/" + FolderName + "/Gravity.csv", header=0)
        # get gyroscope data in x,y,z directions
        df_w = pd.read_csv("SensorLoggerData/" + FolderName + "/Gyroscope.csv", header=0)
        # Get magnetometer data in x,y,z directions
        df_m = pd.read_csv("SensorLoggerData/" + FolderName + "/Magnetometer.csv", header=0)
        # Get true orientation data in yaw pitch, roll.
        df_o = pd.read_csv("SensorLoggerData/" + FolderName + "/Orientation.csv", header=0)
        
        # drop real time
        df_a.drop(columns=['time'], inplace=True)
        # rename columns to a_x, a_y, a_z for accelerometer data
        df_a = df_a.rename(columns={'x': 'a_x', 'y': 'a_y', 'z': 'a_z'})
        
        df_w.drop(columns=['time'], inplace=True)
        df_w = df_w.rename(columns={'x': 'w_x', 'y': 'w_y', 'z': 'w_z'})
        
        df_m.drop(columns=['time'], inplace=True)
        df_m = df_m.rename(columns={'x': 'm_x', 'y': 'm_y', 'z': 'm_z'})
        
        df_o.drop(columns=['time'], inplace=True)
        
        # Merge dataframes based on nearest seconds_elapsed value, since they exactly aren't the same in each dataframe
        df_merged = pd.merge_asof(df_a, df_w, on='seconds_elapsed', tolerance=0.01)
        df_merged = pd.merge_asof(df_merged, df_m, on='seconds_elapsed', tolerance=0.01)
        df_merged = pd.merge_asof(df_merged, df_o, on='seconds_elapsed', tolerance=0.01)
        df_merged.dropna(inplace=True)
        
        # Retrieves the time, angular velocity, acceleration and magnetometer data from the merged dataframe
        # Saves data as attributes of the object
        self.t = df_merged['seconds_elapsed'].values
        self.dt = np.diff(self.t).mean()
        self.w = np.column_stack([df_merged['w_x'], df_merged['w_y'], df_merged['w_z']]) 
        self.a = -np.column_stack([df_merged['a_x'], df_merged['a_y'], df_merged['a_z']]) # a has the opposite sign to match our model
        self.m = np.column_stack([df_merged['m_x'], df_merged['m_y'], df_merged['m_z']])
        # pitch and roll are swapped for the phones real data
        ot_u = np.column_stack([-(df_merged['yaw'] - df_merged['yaw'].iloc[0]), (df_merged['roll'] - df_merged['roll'].iloc[0]), -(df_merged['pitch'] - df_merged['pitch'].iloc[0])]) 
        self.ot = renormalise(ot_u)
        
        self.theta, self.theta_a, self.theta_g, self.zs = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r, ms=self.m)
    
    def _update(self):
        """
        - Overides the _update method from AnalyseMPU since the kalman filter also requires this data
        - Updates the Kalman filter based on the new values in the sliders.
        Args:
            None
        Returns:
            None
        """
        # Handles the sliders logarithmic effects
        self.q = 10**self.s_logq.val
        self.r = 10**self.s_logr.val
        
        # Recalculates the Kalman filter with the new q and r values only self.theta will be updated
        self.theta, self.theta_a, self.theta_g, self.zs = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r, ms = self.m)
        
        # Updates graph data so graph can be redrawn
        self.line_yaw.set_offsets(np.column_stack((self.t, self.theta[:, 0])))
        self.line_pitch.set_offsets(np.column_stack((self.t, self.theta[:, 1])))
        self.line_roll.set_offsets(np.column_stack((self.t, self.theta[:, 2])))
        fig.canvas.draw_idle()
    
    def plot_true(self, fig = None, axs = None, alpha = 0.8):
        """
        Plots the true orientation according to the sensor logger programme
        Args:
            fig (matplotlib.figure.Figure, optional): The figure to plot on. 
            axs (numpy.ndarray, optional): The axes to plot on. If None either fig or axs is None, a new figure and axes will be created.
            alpha (float, optional): Transparency of the points in the scatter plot. Default is 0.8.
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots, to be reused.
        """
        if fig == None and axs == None:
            fig, axs = self.setup_plot()
        axs[0].scatter(self.t, self.ot[:, 0], label='Actual', alpha = alpha, color='black', marker='x', s = 2)
        axs[1].scatter(self.t, self.ot[:, 1], label='Actual', alpha = alpha, color='black', marker='x', s = 2)
        axs[2].scatter(self.t, self.ot[:, 2], label='Actual', alpha = alpha, color='black', marker='x', s = 2)
        return fig, axs
    
    def plot_magnetometer(self, fig = None, axs = None, alpha = 0.8):
        """
        Plots the magnetometer data, specific to AnalysePhone as AnalyseMPU does not process magnetometer data.
        Args:
            fig (matplotlib.figure.Figure, optional): The figure to plot on. 
            axs (numpy.ndarray, optional): The axes to plot on. If None either fig or axs is None, a new figure and axes will be created.
            alpha (float, optional): Transparency of the points in the scatter plot. Default is 0.8.
        Returns:
            fig (matplotlib.figure.Figure): The figure object for the plot.
            axs (numpy.ndarray): An array of axes objects for the subplots, to be reused.
        """
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].scatter(self.t, self.m[:, 0], label='Magnetometer Only', alpha = alpha, color='purple', marker='x', s = 2)
        axs[0].set_ylabel('Magnetometer X')
        axs[0].legend()
        axs[1].scatter(self.t, self.m[:, 1], label='Magnetometer Only', alpha = alpha, color='purple', marker='x', s = 2)
        axs[1].set_ylabel('Magnetometer Y')
        axs[1].legend()
        axs[2].scatter(self.t, self.m[:, 2], label='Magnetometer Only', alpha = alpha, color='purple', marker='x', s = 2)
        axs[2].set_ylabel('Magnetometer Z')
        #axs[2].set_xlabel('Time (s)')
        axs[2].legend()
        fig.tight_layout()
    
class AnalyseTestData(AnalyseMPU):
    """
    Class to read and analyse test data from the ArsGyro and ArsAccel JSON files. From 5IMU example. Inherits from AnalyseMPU.
    """
    def __init__(self):
        """
        - Reads in gyroscope and accelerometer data from JSON files.
        - Converts the data to standard units (m/s^2 for accelerometer and rad/s for gyroscope).
        - Applies a Kalman filter to estimate the orientation based on the gyroscope and accelerometer data.
        Args:
            None
        Returns:
            None
        """
        self.q = 10**-1.6  # Process noise covariance
        self.r = 10**0.7  # Measurement noise covariance
        
        df = pd.read_json('Data/ArsGyro.json')
        data = df["data"]
        data_df = pd.DataFrame(data)

        w_1 = np.array(data_df.loc["wx", "data"])
        w_2 = np.array(data_df.loc["wy", "data"])
        w_3 = np.array(data_df.loc["wz", "data"])
        self.w = np.column_stack((w_1, w_2, w_3)) 

        # For reading accelerometer data from ArsAccel.json
        df = pd.read_json('Data/ArsAccel.json')
        data = df["data"]

        data_df = pd.DataFrame(data)

        a_1 = np.array(data_df.loc["fx", "data"])
        a_2 = np.array(data_df.loc["fy", "data"])
        a_3 = np.array(data_df.loc["fz", "data"])
        self.a = np.column_stack((a_1, a_2, a_3))
        
        self.dt = 0.01
        self.t = np.arange(0, len(self.w) * self.dt, self.dt) 
        
        self.theta, self.theta_a, self.theta_g, self.zs = OrientationKalman.run(self.w, self.a, dt=self.dt, q=self.q, r=self.r)
        
def renormalise(x):
        """
        - Renormalises the orientation data to be in the range [-pi, pi] for yaw and roll, and [-pi/2, pi/2] for pitch.
        - The "true" data from the phone gives yaw [-pi, pi], pitch [-pi, pi] and roll [-pi/2, pi/2] This needs to be converted.
        Args:
            x (numpy.ndarray): The input array of shape (n, 3) the first column is yaw, second is pitch, third is roll.
        Returns:
            numpy.ndarray: The renormalised array of shape (n, 3) with yaw, pitch, and roll in the correct ranges.
        """
        yaws, pitchs, rolls = x[:, 0], x[:, 1], x[:, 2]
        new_thetas = []
        for pitch, yaw, roll in zip(pitchs, yaws, rolls):
            if pitch > (np.pi / 2): # 
                pitch = np.pi - pitch
                yaw += np.pi
                roll += np.pi
            elif pitch < (-np.pi / 2):
                pitch = -np.pi - pitch
                yaw += np.pi
                roll += np.pi
            roll = wrap_pi(roll)
            yaw = wrap_pi(yaw)
            new_thetas.append([yaw, pitch, roll])
        return np.array(new_thetas)
            
def wrap_pi(x):
    """Wraps the input x to be in the range [-pi, pi]."""
    return (x + np.pi) % (2 * np.pi) - np.pi

# Example
# Ensure python is being run from /SummerInternship/7IMUReal using the terminal
# or the IDE is set to use this as the working directory.

test = AnalyseMPU("YawPitchRoll")
alpha = 0.6
fig, axs = test.plot_orientation_from_gyro(alpha = alpha)
fig, axs = test.plot_measurment(fig, axs, alpha = alpha)
fig, axs = test.plot_orientation_fused(fig, axs, alpha = alpha)

# adds legend and x-axis
ax1, ax2, ax3 = axs
ax3.legend(loc = "upper left")
ax3.set_xlabel("Time (s)")
plt.show()
