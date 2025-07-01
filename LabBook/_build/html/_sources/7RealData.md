# Attitude Using Real Data
In this section we explore examples similar to [section 5](5IMU.md) but using real data. We will initially use a gyroscope and an accelarometer, a 6 axis IMU, but will later use a magnetometer aswell, a 9 axis IMU. The 6 axis example used the MPU6050 with an arduino controller and the 9 axis example used a mobile phone app called Sensor Logger which can be downloaded from the [Google Play Store](https://play.google.com/store/apps/details?id=com.kelvin.sensorapp&hl=en-US&pli=1). 

## Implementation

### Raw Data

The data collected can be found [here](https://github.com/MalachiHibbins/IMU/tree/main/7IMUReal/SensorLoggerData). Each folder is a different test and contains the folowing files:

- *In all cases time was measured in seconds.*
- `Gravity.csv`: Contains the gravity vector in the phone's frame of reference. *x,y and z are measured in ms$^{-2}$*
- `Magnetometer.csv`: Contains the magnetometer readings in the phone's frame of reference. *The x and y components are measured in microteslas, the z component is measured in $\mu$T.*
- `Gyroscope.csv`: Contains the gyroscope readings in the phone's frame of reference. *x, y and z are measured in radians rads$^{-1}$*
- `Orientation.csv`: Contains the orientation of the phone in yaw-pitch-roll measured in radians. The programme uses this as the true orientation to compare results agaisnt. *yaw pitch and roll are measured in radians*


The following test files exist:
- `PitchRollCalibration`: Fast movement in the pitch direction followed by a short break, then motion in a combination of the pitch and roll directions followed by a short break, then motion in just the roll direction, this was the same test as in [section 5](5IMU.md) but with data from a real phone. 
- `YawRollPitchCalibration`: Contains data for a calibration test where the phone was moved in each direction followed by a short pause between changing directions. Motion in each direction occurs on its own.
- `FullYaw`: Contains data where the phone was rotated in a full circle in the yaw direction and then rotated back in the opposite direction.
- `FullPitch`: Contains data where the phone was rotated in a full circle in the pitch direction and then rotated back in the opposite direction.

All the other files aren't used in the analysis. The data is collected at a rate of $100$ Hz. The phone's frame of reference is defined as follows:
- The $x$ axis points to the right of the phone.
- The $y$ axis points to the top of the phone.
- The $z$ axis points out of the screen of the phone.

### Programme structure

The programme is structured with three files:
- `AdvKalman.py`: Contains the kalman filter implementation and handles the conversion between quaternions and Euler angles.
- `OrientationKalman.py`: Contains the `run` function which runs the kalman filter on the data and returns the filtered signal, Euler angles from the accelarometer and gyroscope, and the magnetometer readings. It acts as a wrapper for the kalman filter to make it easier to use and contains data such as the parameters and shapes for the kalman fitler.
- `Analysis.py`: Contains the `AnalysePhone` class which is used to read in the data, run the kalman filter and plot the results.



## 9 axis Theory

In [section 5](5IMU.md) gyroscope data and accelarometer data were fused to determine the attitude of the phone. However the accelarometer on its own was undabe to measure the yaw direction. When phone is lying flat in the xy plane we can determine, our update measurment, $\psi^z$, accurately using the magnetometer.
```{math}
:label: eq-Magnetometer
\psi^z = \arctan\left(\frac{m_y}{m_x}\right)
```
Where $m_x$ and $m_y$ are the x and y components of the magnetometer reading. The magnetometer reading is in the phone's frame of reference. However most of the time the phone is not lying flat in the xy plane, so we will use the accelarometer data to write the corrected magnetometer readings $m_x'$ and $m_y'$.

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
For the kalman filter to work the the attitude measurments must be normalised. If there was multiple ways to write the attitude then the kalman filter would not work. For example if the yaw was written as $-\pi$ and $\pi$ then the kalman filter would not be able to determine which one was correct. So we will normalise the yaw and roll to be in the range and $[-\pi, \pi]$ and pitch to be in the range $[-\pi/2, \pi/2]$. 
```

#### MPU6050 6-axis IMU

```{figure} image-39.png
:name: fig-fast-yaw-roll-pitch-6-axis
Fast oscillations in the yaw, roll and pitch directions. The orange line is the measurments from integrating the angular velocities from the gyro and red line is the measurements from the magnetometer and accelarometer. The blue line is the kalman filtered attitude which fuses data from the gyroscope and accelarometer/magnetometer. 
```

The drift is clearly visible in the gryo measurments which would make it unsuitable for accurate measurement. But the accelarometer measurments seem to be much more accurate. However it is unclear if kalman filter improves on the accelaroeter measurments. The only place the gyroscope is better is in the yaw direction as the accelerometer is unable to measure this.

```{figure} image-40.png
:name: fig-fast-yaw-roll-pitch-6-axis-tuned
The same data as {numref}`fig-fast-yaw-roll-pitch-6-axis` but with the kalman filter tuned with increased weighting on the prediction, the gyro data.
```

Here the kalman filter has a much better shape for the yaw direction since there is a greater weighting on the gyro data meaning oscilaltions are much more pronounced. The kalman filter also still filters out the drift from the gyro data making it a very accurate fit.

```{figure} image-43.png
:name: fig-fast-yaw-roll-pitch-6-axis-zoomed
Zoomed in on the pitch oscillations in {numref}`fig-fast-yaw-roll-pitch-6-axis`.
```

There is a problem in the oscillations in all directions are now delayed as the gyroscope is slower to to changes than the accelearometer, hence by putting more faith in the prediction although the fit better resembles the shape of the true data it is slightly delayed. 

It is also clear that the kalman filter is able to filter out some of the noise from the accelarometer data. The accelarometer data is typically noisier than the gyroscope data, so putting more emphasis on the gryoscope will help to filter out noise. 

To minimise the lag the weighting on the prediction would need reducing but this would incrase the noise present in the kalman filter estimate. The easiest way to improve the fit would be to indtruduce a magnetometer as this would reduce the lag in the yaw direction and improve the fit.

#### Phone, 9-axis IMU


```{figure} image-32.png
:name: fig-fast-yaw-roll-pitch
Fast oscillations in the yaw, roll and pitch directions. The orange line is the measurments from integrating the angular velocities from the gyro and red line is the measuremnts from the magnetometer and accelarometer. The blue line is the kalman filtered attitude which fuses data from the gyroscop and accelarometer/magnetometer.  The black line is the phones own attitude measurments.
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

For the yaw direction both sets of instruments are indicating the correct shape but there is drift especially in the gyro. There is interference in the pitch direction, again the kalman filter does a good job of filtering this out. The gyro only singal has disappeared due to a invalid value being entered. Since each value in the kalman filter rleies on the previous value if one value is invalid then all subsequent values will be invalid. 

#### Full Yaw

```{figure} image-37.png
:name: fig-full-yaw
The phone was rotated in a full circle in the yaw direction and then rotated back in the opposite direction. The orange line is the measurments from integrating the angular velocities from the gyro and red line is the measuremnts from the magnetometer and accelarometer. The blue line is the kalman filtered attitude which fuses data from the gyroscope and accelarometer/magnetometer.  The black line is the phones own attitude measurments.
```

The kalman filter is able to reduce noise in the magnetometer measurments but the gyroscope seems to provide the best fit here. This could be because the magnetometer is overly sensetive. However it is likely that the phone reies more on its gyroscope for measurments of orientation since magnetometer data isn't as reliable and is sometimes calibrated poorly. Whereas a gyroscope with a small amount of drift will always work.