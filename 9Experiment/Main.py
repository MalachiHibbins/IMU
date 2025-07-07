import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.signal import savgol_filter
from matplotlib.widgets import Slider


# 10 colors inspired by plasma colormap (purple -> pink -> yellow progression)
colors = ['black', 'gold', 'darkorange', 'orangered', 'red', 'crimson',
          'darkviolet', 'blueviolet', 'indigo', 'darkblue',
          'navy']

def normalise_angle(angle, offset = 0):
    """
    Normalizes an angle to the range [-pi, pi].
    args:
        angle (list): The angle before normalistaion
        offset (flaot): The initial angle offset to be obstructed so that it starts from zero
    """
    angle = np.array(angle)
    
    return ((angle - offset + np.pi) % (2 * np.pi)) - np.pi

def integrate_for_theta(dt, w, theta0 = 0, c = 20):
    """
    Integrates angular velocity to get orientation.
    args:
        dt (float): Time step between measurements.
        w (list): Angular velocity measurements.
        theta0 (float): Initial angle, default is 0.
        c (int): Number of samples to use for offset calculation, default is 20.
    """
    theta = np.zeros(len(w))
    theta[0] = theta0
    for i in range(1, len(w)):
        theta[i] = theta[i-1] + dt * w[i-1]
    offset = theta[:c].mean()  # Use first 20 samples to calculate offset
    theta = ((theta - offset) + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
    return theta

def EMALowPass(signal, alpha): # Exponential Moving Average Low Pass
    """
    Applies an EMA to the low pass filter see section 3.
    Args:
        signal (list): The signal to be filtered.
        alpha (float): The smoothing factor determines how quickly weightings of previous terms decay (exponentially)
    """
    m_prev = signal[0]
    filtered_signal = [m_prev]
    
    for m in signal:
        m_new = alpha * m_prev + (1 - alpha) * m
        filtered_signal.append(m_new)
        m_prev = m_new
    return filtered_signal[1:]

def EMAHighPass(signal, alpha): # Exponential Moving Average High Pass
    """
    Applies an EMA to the high pass filter using the opposite method as EMALowPass.
    Args:
        signal (list): The signal to be filtered.
        alpha (float): The smoothing factor determines how quickly weightings of previous terms decay (exponentially)
    """
    m_prev = signal[0]
    filtered_signal = [m_prev]
    
    for i in range(1, len(signal)):
        m_new = alpha * (m_prev + signal[i] - signal[i-1])
        filtered_signal.append(m_new)
        m_prev = m_new
    return filtered_signal      
     
def calculate_magnetometer_angle(m, c=20):
    mx, my, mz = m[:, 0], m[:, 1], m[:, 2]
    theta = -np.arctan2(my, mx)
    offset = theta[:c].mean()
    theta = normalise_angle(theta, offset)
    return theta

def diff(x, y):
    """
    Calculates the difference between two arrays using the cyclic nature of angles between -pi and pi.
    arg:
    - x (float): First array of angles.
    - y (float): Second array of angles.
    Returns:
    - difference (float): The cyclic difference between the two arrays.
    """
    
    difference = x - y
    if difference > np.pi:
        difference -= 2 * np.pi
    elif difference < -np.pi:
        difference += 2 * np.pi    
    
    return difference

def kalman_filter(zs, us, x_e, A, B, P, H, Q, R, R_u):
    x_es = []
    P_es = []
    for z, u in zip(zs, us):
        x = x_e
        # Predict state error
        x_p = A*x + B*u
        P_p = A*P*A + B*R_u*B + Q
        
        # Predict kalman gain
        K_k = P_p*H*(H*P_p*H+R)**-1
        # Correct state and error covariance
        x_e = x_p + K_k * diff(z, H*x_p)
        P_e = P_p - K_k*H*P_p
        
        x_es.append(x_e)
        P_es.append(P_e)
    return normalise_angle(x_es), P_es

class AnalysePhone(): #
    """
    - Reads and analyses data from the sensorlogger app on a phone.
    """
    def __init__(self, FolderName, Saved_Params = None, **kalman_kwargs):
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
        if Saved_Params is None:
            self.alpha_EMAHP = 0.5
            self.alpha_EMALP = 0.5
            self.window_length = 21
            self.poly_order = 3  
            self.Q = 0.05
            self.R = 0.001
            self.R_m = 0.001
            self.c = 20
        else:
            df_params = pd.read_csv(Saved_Params, header=None, index_col=0, sep=':')
            self.alpha_EMAHP = df_params.loc['alpha_EMAHP'].values[0]
            self.alpha_EMALP = df_params.loc['alpha_EMALP'].values[0]
            self.c = int(df_params.loc['c'].values[0])
            self.window_length = int(df_params.loc['window_length'].values[0])
            self.poly_order = int(df_params.loc['poly_order'].values[0])
            self.Q = float(df_params.loc['Q'].values[0])
            self.R = float(df_params.loc['R'].values[0])
            self.R_m = float(df_params.loc['R_u'].values[0])
            
        
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
        df_merged = pd.merge_asof(df_w, df_m, on='seconds_elapsed', tolerance=0.01)
        df_merged = pd.merge_asof(df_merged, df_o, on='seconds_elapsed', tolerance=0.01)
        df_merged.dropna(inplace=True)
        
        # Retrieves the time, angular velocity, acceleration and magnetometer data from the merged dataframe
        # Saves data as attributes of the object
        self.t = df_merged['seconds_elapsed'].values
        self.dt = np.diff(self.t).mean()
        self.w = df_merged['w_z'].values
        self.m = np.column_stack([df_merged['m_x'], df_merged['m_y'], df_merged['m_z']])
        # pitch and roll are swapped for the phones real data
        offset = df_merged['yaw'][:self.c].mean()
        self.theta_phone = -(((df_merged['yaw'] - offset) + np.pi) % (2 * np.pi) - np.pi)

        # Integrate to calcualte theta
        self.theta_integrated = integrate_for_theta(self.dt, self.w, self.dt)
        # Calculate magnetometer angle
        self.theta_magnetometer = calculate_magnetometer_angle(self.m)
        # Kalman filter on data
        self.theta_kalman, self.theta_kalman_var = kalman_filter(self.theta_magnetometer, self.w, B = self.dt, Q = self.Q, R = self.R, R_u = self.R_m, **kalman_kwargs)
        # LowPass filter on integrated data
        self.theta_integrated_HP = EMAHighPass(self.theta_integrated, self.alpha_EMAHP)
        # LowPass filter on magnetometer data
        self.theta_magnetometer_LP = EMALowPass(self.theta_magnetometer, self.alpha_EMALP)
        # salvgov filter on integrated data
        self.theta_magnetometer_sav = savgol_filter(self.theta_magnetometer, window_length=self.window_length, polyorder=self.poly_order)
        
        
        self.filtered_data = [self.theta_phone, self.theta_magnetometer, self.theta_magnetometer_LP,
                              self.theta_integrated, self.theta_integrated_HP, self.theta_kalman, self.theta_magnetometer_sav]
        self.filtered_data_names = ['Phone, Builtin Filter', 'Magnetometer, No Filter', 'Magnetometer, High Pass Filter',
                                    'Gyro, No Filter', 'Gyro, Low Pass Filter', 'Fusion, Kalman Filter', 'Magnetometer, Savitzky-Golay Filter']
        
    def calculate_correlation_matrix(self):
        """
        - Calculates the correlation matrix of the data.
        - Returns the correlation matrix as a pandas dataframe.
        """
        df = pd.DataFrame({
            'Fusion, KF': self.theta_kalman,
            'Gyro, None': self.theta_integrated,
            'Gyro, LPF': self.theta_integrated_HP,
            'Magn, None': self.theta_magnetometer,
            'Magn, HPF': self.theta_magnetometer_LP,
            'Magn, GF': self.theta_magnetometer_sav,
            'Built in Filters': self.theta_phone
        })
        return df.corr()
    
    def calculate_MSE(self):
        df = pd.DataFrame({
            'Fusion, KF': self.theta_kalman,
            'Gyro, None': self.theta_integrated,
            'Gyro, LPF': self.theta_integrated_HP,
            'Magn, None': self.theta_magnetometer,
            'Magn, HPF': self.theta_magnetometer_LP,
            'Magn, GF': self.theta_magnetometer_sav,
            'Built in Filters': self.theta_phone
        })
        # Calculate MSE between each column and self.theta_phone
        mse_values = ((df.sub(self.theta_phone, axis=0)) ** 2).mean()
        return mse_values[:-1]
    
    def plot_phone_correlations(self, bar_fig = None, bar_ax = None):
        corr_matrix_squared = ((self.calculate_correlation_matrix())**2)
        corr_phone_squared = corr_matrix_squared.iloc[-1, :-1]
        
        if bar_fig is None or bar_ax is None:
            bar_fig, bar_ax = plt.subplots(figsize=(8, 5))
        else:
            bar_ax.clear()
        
        self.bars = sns.barplot(x=corr_phone_squared.index, y=corr_phone_squared.values, ax=bar_ax, palette=colors[1:])
        bar_ax.set_ylabel('Correlation Coefficient Squared')
        bar_ax.set_xlabel('Filter')
        plt.setp(bar_ax.get_xticklabels(), rotation=15, ha='right')
        bar_fig.tight_layout()
        
    def plot_phone_MSEs(self, bar_fig2 = None, bar_ax2 = None):
        mse_values = self.calculate_MSE()
        if bar_fig2 is None or bar_ax2 is None:
            bar_fig2, bar_ax2 = plt.subplots(figsize=(8, 5))
        else:
            bar_ax2.clear()
        
        self.bars2 = sns.barplot(x=mse_values.index, y=mse_values.values, ax=bar_ax2, palette=colors[1:])
        bar_ax2.set_ylabel('Mean Squared Error')
        bar_ax2.set_xlabel('Filter')
        plt.setp(bar_ax2.get_xticklabels(), rotation=15, ha='right')
        bar_fig2.tight_layout()
        
        
        
        
    def plot_all(self, fig = None, ax = None, alpha = 0.5):
        R = len(self.filtered_data)
        N = R // 2
        self.lines = [None] * R
        length = 2 * N
        fig, axs = plt.subplots(N, 2, figsize=(8, length), sharex=True, sharey=True)
        axs = axs.flatten()
        for i in range(1, R):
            i_ = i - 1
            self.lines[i], = axs[i_].plot(self.t, self.filtered_data[i], label=self.filtered_data_names[i],
                        alpha=alpha, color = colors[i], linewidth = 2)
            axs[i_].plot(self.t, self.filtered_data[0], label=self.filtered_data_names[0],
                        alpha=alpha, color = colors[0], linewidth = 2)
            axs[i_].hlines(np.pi, 0, max(self.t), color='black', linestyle='--', linewidth=0.5)
            axs[i_].hlines(-np.pi, 0, max(self.t), color='black', linestyle='--', linewidth=0.5)
            axs[i_].legend()
            if i > R-3:
                axs[i_].set_xlabel('Time (s)')
            if i_ % 2 == 0:
                axs[i_].set_ylabel('Yaw (rad)')
            for j in range(1, R):
                if i != j:
                    axs[i_].plot(self.t, self.filtered_data[j], label=self.filtered_data_names[j],
                                alpha=alpha/3, color = colors[j], linewidth = 0.5)
            
        fig.tight_layout()
        # Then adjust the bottom margin
        fig.subplots_adjust(bottom=0.25)
        
        self.set_up_sliders(fig, length = 0.325, height = 0.03)
        self.s_logQ.on_changed(lambda val: self._update())
        self.s_logR_g.on_changed(lambda val: self._update())
        self.s_logR_m.on_changed(lambda val: self._update())
        self.s_alpha_HP.on_changed(lambda val: self._update())
        self.s_alpha_LP.on_changed(lambda val: self._update())
        self.s_window_length.on_changed(lambda val: self._update())
        self.s_poly_order.on_changed(lambda val: self._update())
        
    def _update(self):
        """
        - Updates the data based on the slider values.
        - Recalculates the filtered data and updates the plots.
        """
        self.Q = 10**self.s_logQ.val
        self.R = 10**self.s_logR_g.val
        self.R_m = 10**self.s_logR_m.val
        self.alpha_EMAHP = self.s_alpha_HP.val
        self.alpha_EMALP = self.s_alpha_LP.val
        self.window_length = int(self.s_window_length.val)
        self.poly_order = int(self.s_poly_order.val)
        
        # Integrate to calcualte theta
        self.theta_integrated = integrate_for_theta(self.dt, self.w, self.dt)
        # Calculate magnetometer angle
        self.theta_magnetometer = calculate_magnetometer_angle(self.m)
        # Kalman filter on data
        self.theta_kalman, self.theta_kalman_var = kalman_filter(self.theta_magnetometer, self.w, B = self.dt,
                                                                 Q = self.Q, R = self.R, R_u = self.R_m, **kalman_kwargs)
        # LowPass filter on integrated data
        self.theta_integrated_HP = EMAHighPass(self.theta_integrated, self.alpha_EMAHP)
        # LowPass filter on magnetometer data
        self.theta_magnetometer_LP = EMALowPass(self.theta_magnetometer, self.alpha_EMALP)
        # salvgov filter on integrated data
        self.theta_magnetometer_sav = savgol_filter(self.theta_magnetometer, window_length=self.window_length, polyorder=self.poly_order)
        
        self.filtered_data = [self.theta_phone, self.theta_magnetometer, self.theta_magnetometer_LP,
                              self.theta_integrated, self.theta_integrated_HP, self.theta_kalman, self.theta_magnetometer_sav]

        self.lines[1].set_ydata(self.theta_magnetometer)
        self.lines[2].set_ydata(self.theta_magnetometer_LP)
        self.lines[3].set_ydata(self.theta_integrated)
        self.lines[4].set_ydata(self.theta_integrated_HP)
        self.lines[5].set_ydata(self.theta_kalman)
        self.lines[6].set_ydata(self.theta_magnetometer_sav)
        
        self.plot_phone_correlations(self.bars.figure, self.bars.axes)
        self.plot_phone_MSEs(self.bars2.figure, self.bars2.axes)
        self.bars.figure.canvas.draw_idle()
        self.bars2.figure.canvas.draw_idle()
        
        
    def set_up_sliders(self, fig, length = 0.375, height = 0.03):
        # Left column - moved up a bit
        left = 0.1
        right = 0.55
        ax_Q = fig.add_axes([left, 0.15, length, height], facecolor=colors[5])
        ax_R_g = fig.add_axes([left, 0.11, length, height], facecolor=colors[5])
        ax_R_m = fig.add_axes([left, 0.07, length, height], facecolor=colors[5])
        
        # Right column - moved up to match left column
        ax_Alpha_HP = fig.add_axes([right, 0.15, length, height], facecolor=colors[2])
        ax_Alpha_LP = fig.add_axes([right, 0.11, length, height], facecolor=colors[4])
        ax_window_length = fig.add_axes([right, 0.07, length, height], facecolor=colors[6])
        ax_poly_order = fig.add_axes([right, 0.03, length, height], facecolor=colors[6])
        
        self.s_logQ = Slider(ax_Q, '$\log(Q)$', -3, 5, valinit=np.log10(self.Q), valstep=0.1)
        self.s_logR_g = Slider(ax_R_g, '$\log(R^g)$', -3, 5, valinit=np.log10(self.R), valstep=0.1)
        self.s_logR_m = Slider(ax_R_m, '$\log(R^m)$', -3, 5, valinit=np.log10(self.R_m), valstep=0.1)
        self.s_alpha_HP = Slider(ax_Alpha_HP, '$\\alpha^{LP}$', 0, 1, valinit=self.alpha_EMAHP, valstep=0.02)
        self.s_alpha_LP = Slider(ax_Alpha_LP, '$\\alpha^{HP}$', 0, 1, valinit=self.alpha_EMALP, valstep=0.02)
        self.s_window_length = Slider(ax_window_length, '$W$', 3, 51, valinit=self.window_length, valstep=2)
        self.s_poly_order = Slider(ax_poly_order, '$\mathbb{O}$', 1, 5, valinit=self.poly_order, valstep=1)
        
    def save_params(self, filename):
        """
        - Saves the parameters of the object to a file.
        Args:
            filename (str): Name of the file to save the parameters to.
        Returns:
            None
        """
        params = {
            'alpha_EMAHP': self.alpha_EMAHP,
            'alpha_EMALP': self.alpha_EMALP,
            'c': self.c,
            'window_length': self.window_length,
            'poly_order': self.poly_order,
            'Q': self.Q,
            'R': self.R,
            'R_u': self.R_m
        }
        with open(filename, 'w') as f:
            for key, value in params.items():
                f.write(f"{key}: {value}\n")

kalman_kwargs = {"P" : 0.1, "H" : 1, "A" : 1.0, "x_e" : 0.0}
test_oscillations = AnalysePhone("Oscillations3", Saved_Params="saved_params.txt", **kalman_kwargs)
test_oscillations.plot_all()
test_oscillations.plot_phone_correlations()
test_oscillations.plot_phone_MSEs()

plt.show()
test_oscillations.save_params("saved_params.txt")

test_rotations = AnalysePhone("Rotations1", Saved_Params="saved_params.txt", **kalman_kwargs)
test_rotations.plot_all()
test_rotations.plot_phone_correlations()
test_rotations.plot_phone_MSEs()

plt.show()
test_rotations.save_params("saved_params.txt")
