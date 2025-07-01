# Attitude Using Real Data
This section explores examples similar to [section 5](5IMU.md) but using real data. Initially useing a gyroscope and an accelarometer, a 6 axis IMU, and later useing a magnetometer aswell, a 9 axis IMU. The 6 axis example uses the MPU6050 with an arduino controller and the 9 axis example used the mobile phone sensors with the [sensor logger](https://play.google.com/store/apps/details?id=com.kelvin.sensorapp&hl=en-US&pli=1) app. 

## Implementation
The code for this section can be found [here](https://github.com/MalachiHibbins/IMU/tree/main/7IMUReal). The main file is `Analysis.py` which contains `AnalyseMPU` class which is the parent class for `AnalysePhone` and `AnalyseTest`, these read data from the MPU6050, sensor logger and test data (from [section 5](5IMU.md)) respectively.

### Raw Data from MPU6050

Each file with measurments came with its own calibration file, which was used to determine the offset in each direction. Different tests were performed with the MPU6050, the data for these are contained in the folders:
- `YawPitchRoll`: Involves fast oscillations in the yaw direction followed by a short break, then oscillations in the roll direction followed by a short break, then oscillations in the pitch direction.
- `FullYaw`: Contains data where the MPU6050 was rotated in a full circle in the yaw direction and then rotated back in the opposite direction.
- `FullPitch`: Contains data where the MPU6050 was rotated in a full circle in the pitch direction and then rotated back in the opposite direction.

```{warning}
Currently the kalman filter will only correctly fit `YawPitchRoll` data. When the sensor was rotated beyond $180^o$ in the yaw and roll directions or beyond $90^o$ in the pitch direction the kalman filter will need to renormalise the attitude, which made it difficult to fit the kalman filter. This wasn't a problem with `YawPitchRoll` data as no full rotations were performed.
```

### Raw Data From Sensor Logger

The data collected can be found [here](https://github.com/MalachiHibbins/IMU/tree/main/7IMUReal/SensorLoggerData). Each folder contains a different test and contains the folowing files:

- *In all cases time was measured in seconds.*
- `Gravity.csv`: Contains the accelaration readings without the accelaration due to gravity removed. *The x, y and z components are measured in ms$^{-2}$*.
- `Magnetometer.csv`: Contains the magnetometer readings in the phone's frame of reference. *The x, y and z components are measured in $\mu$T.*
- `Gyroscope.csv`: Contains the gyroscope readings in the phone's frame of reference. *x, y and z are measured in radians rads$^{-1}$*
- `Orientation.csv`: Contains the orientation of the phone in yaw-pitch-roll measured in radians. We will refer to this as the true orientation as its the orientation calculated by the phone.
All other files are not used in this analysis.


Tests were performed and stored in the folders:
- `PitchRollCalibration`: Fast movement in the pitch direction followed by a short break, then motion in a combination of the pitch and roll directions followed by a short break, then motion in just the roll direction, this was the same test as in [section 5](5IMU.md).
- `YawRollPitchCalibration`: Contains data for a calibration test where the phone was moved in each direction alone followed by a short pause. 
- `FullYaw`: Contains data where the phone was rotated in a full circle in the yaw direction and then rotated back in the opposite direction.
- `FullPitch`: Contains data where the phone was rotated in a full circle in the pitch direction and then rotated back in the opposite direction.

### Programme structure

The programme is structured with three files:
- `AdvKalman.py`: Contains the kalman filter implementation and handles the conversion between quaternions and Euler angles.
- `OrientationKalman.py`: Contains the `run` function which runs the kalman filter on the data and returns the filtered signal, Euler angles from the accelarometer and gyroscope, and the magnetometer readings. It acts as an interface between Analysis.py and AdvKalman.py.
- `Analysis.py`: Contains the `AnalysePhone` class which is used to read in the data, run the kalman filter and plot the results.



## Theory, 9 axis

In [section 5](5IMU.md) gyroscope data and accelarometer data were fused to determine the attitude of the phone. However the accelarometer on its own was undabe to measure the yaw direction. When phone is lying flat in the xy plane we can determine, our update measurment, $\psi^z$, accurately using the magnetometer.
```{math}
:label: eq-Magnetometer
\psi^z = \arctan\left(\frac{m_y}{m_x}\right)
```
Where $m_x$ and $m_y$ are the x and y components of the magnetometer reading. The magnetometer reading is in the phone's frame of reference this is normally not the xy plane so the accelarometer data to is required to write the corrected magnetometer readings $m_x'$ and $m_y'$.

```{math}
:label: eq-gyro-correction
m_x^{'} = m_x \cos(\theta^z) + m_y \sin(\theta^z) \sin(\phi^z) + m_z \sin(\theta^z) \cos(\phi^z)\\
m_y^{'} = m_y \cos(\theta^z) - m_x \sin(\theta^z) \sin(\phi^z) + m_z \sin(\theta^z) \cos(\phi^z)\\
```

Where $\phi^z$ and $\theta^z$ are the roll and pitch angles calculated using the accelarometer.

Then as before we can use our corrected values to determine $\psi^z$ when the magnetometer isn't lying flat in the xy plane. To summarise the update measurment will be formed from the accelarometer, for pitch and roll and the magnetometer for yaw. The prediction step will use measurments from the gyroscope in the same way as [section 5](5IMU.md).

## Results

### Fast Yaw Pitch Roll
These examples involved fast oscillations in the yaw directions followed by a short break, then oscillations in the roll direction followed by a short break, then oscillations in the pitch direction. 

```{note}
Here yaw-pitch-roll is normalised as follows: yaw is in the range $[-\pi, \pi]$, pitch is in the range $[-\pi/2, \pi/2]$ and roll is in the range $[-\pi, \pi]$. 
```

#### MPU6050 6-axis IMU

So that the correction was normalised the correction in the yaw direction was set to always be zero, the pitch and roll directions came from acelarometer measurments. The prediction came from gyroscope measurments.

```{figure} image-39.png
:name: fig-fast-yaw-roll-pitch-6-axis
Fast oscillations in the yaw, roll and pitch directions. The orange line is the measurments from integrating the angular velocities from the gyro and red line is the measurements from the accelarometer. The blue line is the kalman filtered attitude which fuses data from the gyroscope and accelarometer/magnetometer. 
```

The drift is clearly visible from the gyro. But the accelarometer measurments seem to be much more accurate. However it is unclear if kalman filter improves on the accelaroeter measurments with the current $Q$ and $R$. 

```{figure} image-40.png
:name: fig-fast-yaw-roll-pitch-6-axis-tuned
The same data as {numref}`fig-fast-yaw-roll-pitch-6-axis` but with the kalman filter tuned, with larger $R$ and smaller $Q$, increasing the weighting on the prediction, the gyro data.
```

Here the kalman filter has a much better shape for the yaw direction since there is a greater weighting on the prediction meaning oscilaltions from the gyro are much more pronounced in the kalman filter. The kalman filter also still filters out the drift from the gyro data making it a very accurate fit.

```{figure} image-43.png
:name: fig-fast-yaw-roll-pitch-6-axis-zoomed
Zoomed in on the pitch oscillations in {numref}`fig-fast-yaw-roll-pitch-6-axis`.
```

There is a problem in the oscillations in all directions are now delayed as the gyroscope is slower to to changes than the accelearometer, hence by putting more faith in the prediction although the fit more accurately resembles the shape of the true data it is slightly delayed. 

The kalman filter is able to filter out some of the noise from the accelarometer data. The accelarometer data is typically noisier than the gyroscope data, so putting more emphasis on the gryoscope will help to filter out noise. 

#### Phone, 9-axis IMU
Introducing magnetometer measurments means there is 2 measurments for each axis, reducing the total drift.

```{figure} image-32.png
:name: fig-fast-yaw-roll-pitch
Fast oscillations in the yaw, roll and pitch directions. The orange line is the measurments from integrating the angular velocities from the gyro and red line is the measuremnts from the magnetometer and accelarometer. The blue line is the kalman filtered attitude which fuses data from the gyroscope and accelarometer/magnetometer.  The black line is the phones own attitude measurments.
```

There is significant drift in the meausmrents from the gyro alone the end measurments from the gyro alone has drifted by approximately $60^o$ from the true value where as the kalman filter and the magnetometer/accelarometer data are much better fits. Also the amplitude of the measurments oscillations in the roll direction is much smaller than the yaw and pitch directions. This was because its because of the way the phone had to be held.

```{figure} image-34.png
:name: fig-fast-yaw-roll-pitch-2
Zoomed in on the yaw oscillations in {numref}`fig-fast-yaw-roll-pitch`. 
```

Here the kalman filter is able to corret for both the drift from the gyro and noise from the magnetometer making it a very good fit for yaw. The is interference with the gyroscope in the pitch direction but the kalman filter sucessfully filters this out. 

```{figure} image-35.png
:name: fig-fast-yaw-roll-pitch-2
Zoomed in on the pitch oscillations in {numref}`fig-fast-yaw-roll-pitch`.
```

There is interference in the magnetometer/gyroscope measurments but the kalman filter filters this out sucessfully but this does cause drift in the kalman filter measurments. The gyroscope measurments for Pitch are incredably noisy possibly due to oversensitivty of the gyroscope in the phone, but the kalman filter remains a good fit. The Roll direction remains fairly stable.

```{figure} image-36.png
:name: fig-fast-yaw-roll-pitch-2
Zoomed in on the roll oscillations in {numref}`fig-fast-yaw-roll-pitch`.
```

For the yaw direction both sets of instruments are indicating the correct shape but there is drift especially in the gyro. There is interference in the pitch direction, again the kalman filter does a good job of filtering this out. Its unclear if the kalman filter has benefitte

### Full Rotation, Yaw

```{figure} image-37.png
:name: fig-full-yaw
The phone was rotated in a full circle in the yaw direction and then rotated back in the opposite direction. The orange line is the measurments from integrating the angular velocities from the gyro and red line is the measuremnts from the magnetometer and accelarometer. The blue line is the kalman filtered attitude which fuses data from the gyroscope and accelarometer/magnetometer.  The black line is the phones own attitude measurments.
```

The kalman filter is able to reduce noise in the magnetometer measurments but the gyroscope seems to provide the best fit here. This could be because the magnetometer is overly sensetive. However it is likely that the phone reies more on its gyroscope for measurments of orientation since magnetometer data isn't as reliable and is sometimes calibrated poorly. Whereas a gyroscope with a small amount of drift will always work.

```{note}
I didn't have enough time during the placement to test the kalman filter for full rotations.
```
