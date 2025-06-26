## IMU Real
These examples are similar to section 5, except rather than using the data from the Dr Shane Ross videos use real data from the MPU6050 or mobile phone with the sensor logger app ([text](https://play.google.com/store/apps/details?id=com.kelvin.sensorapp&hl=en-US&pli=1. 

Analysis.py - Calls functions from OrientationKalman and AdvKalman.py and plots graphs. Two objects are present AnalysePhone and AnalyseMPU. Initialising these objects reads in the data. Most of the methods are for plotting graphs. **AnalysePhone reads data from a folder whereas AnalyseMPU reads from just a file**. AnalyseMPU requires two filenames to be entred one for calibration and one for measurments. The calibration should involve taking measurments while the MPU6050 is stationary. The app aultomatically carries out a calibration. If you want data to be plotted onto the same graph as another pass fig and ax to the function. By default is none so will generate its own fig and ax.

AdvKalman.py - Handles all kalman filter mathematics e.g. converting to quanternians and filtering algorithm.

OrientationKalman - Handles different kalman filters, e.g. running kalman filter with correction mode off to calculate the attitude from gyroscope data only, running the full kalman filter, and getting attitude measurments from accelarometer alone. 

## Examples


