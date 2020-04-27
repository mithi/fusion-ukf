# [🐳](https://mithi.github.io/deep-blueberry) [☕️](https://ko-fi.com/minimithi) [🧧](https://www.paypal.me/minimithi)

# INTRODUCTION
This is an unscented Kalman Filter implementation in C++ for fusing lidar and radar sensor measurements.
A Kalman filter can be used anywhere where you have uncertain information about some dynamic system, 
and you want to make an educated guess about what the system is going to do next. 

**In this case, we have two 'noisy' sensors:**
- A lidar sensor that measures a tracked object's position in cartesian-coordinates `(x, y)`
- A radar sensor that measures a tracked object's position and relative velocity (the velocity within line of sight) in polar coordinates `(rho, phi, drho)`

**We want to predict a tracked object's position, how fast it's going in what direction, and how fast it is
turning (yaw rate) at any point in time.** 
- In essence we want to get: the position of the system in cartesian coordinates, the velocity magnitude, the yaw angle in radians, and yaw rate  in radians per second `(x, y, v, yaw, yawrate)`
- We are assuming a **constant turn/yaw rate and velocity magnitude model** (CRTV) for this particular system

**This unscented kalman filter does just that.** 

- NOTE: Compared with an [Extended Kalman Filter](https://github.com/mithi/Fusion-EKF-CPP) with a constant velocity model, RMSE should be lower for the unscented Kalman filter especially for velocity. The CTRV model is more precise than a constant velocity model. And UKF is also known for handling non-linear equations better than EKF.
- [Harvard Paper about UKF](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)

-----
# CONTENTS
- Basic Usage
- Notes

-----
# BASIC USAGE
- Dependencies are same as in [here](https://github.com/mithi/fusion-ekf)
- Clone this repository 
```
$ git clone https://github.com/mithi/fusion-ukf/
```
- Go inside the `build` folder and compile: 
```
$ cd build
$ CC=gcc-6 cmake .. && make
```

- To execute inside the `build` folder use the following format: 

```
$ ./unscentedKF /PATH/TO/INPUT/FILE /PATH/TO/OUTPUT/FILE
$ ./unscentedKF ../data/data-3.txt ../data/out-3.txt
```

- Please use the following format for your input file
```
L(for lidar) m_x m_y t r_x r_y r_vx r_vy, r_yaw, r_yawrate
R(for radar) m_rho m_phi m_drho t r_px r_py r_vx r_vy, r_yaw, r_yawrate

Where:
(m_x, m_y) - measurements by the lidar
(m_rho, m_phi, m_drho) - measurements by the radar in polar coordinates
(t) - timestamp in unix/epoch time the measurements were taken
(r_x, r_y, r_vx, r_vy, r_yaw, r_yawrate) - the real ground truth state of the system

Example:
L 3.122427e-01  5.803398e-01  1477010443000000  6.000000e-01  6.000000e-01  5.199937e+00  0 0 6.911322e-03
R 1.014892e+00  5.543292e-01  4.892807e+00  1477010443050000  8.599968e-01  6.000449e-01  5.199747e+00  1.796856e-03  3.455661e-04  1.382155e-02
```
- The program outputs the predictions in the following format on the output file path you specified:
```
time_stamp  px_state  py_state  v_state yaw_angle_state yaw_rate_state  sensor_type NIS px_measured py_measured px_ground_truth py_ground_truth vx_ground_truth vy_ground_truth
1477010443000000  0.312243  0.58034 0 0 0 lidar 2.32384e-319  0.312243  0.58034 0.6 0.6 0 0
1477010443050000  0.735335  0.629467  7.20389 9.78669e-18 5.42626e-17 radar 74.6701 0.862916  0.534212  0.859997  0.600045  0.000345533 4.77611e-06
...
```

-----
# NOTES

### If you take a look at [settings](https://github.com/mithi/Fusion-UKF-CPP/blob/master/headers/settings.h) you'll see the following:
```
//process noise standard deviations
const double STD_SPEED_NOISE = 0.9; // longitudinal acceleration in m/s^2
const double STD_YAWRATE_NOISE = 0.6; // yaw acceleration in rad/s^2
```

### Here's the terminal output from the given data set
![terminal output](https://github.com/mithi/Fusion-UKF-CPP/blob/master/images/terminal_output.png)

### Here's a visualization of how it's performing
![Visualization](https://github.com/mithi/Fusion-UKF-CPP/blob/master/images/graph-1.png)

###  Here's a visualization of the Radar's NIS
![Radar's NIS](https://github.com/mithi/Fusion-UKF-CPP/blob/master/images/radar_nis.png)

### Here's a visualization of the Lidar's NIS
![Lidar's NIS](https://github.com/mithi/Fusion-UKF-CPP/blob/master/images/lidar_nis.png)

### Here's my UKF algorithm overview
![UKF Algorithm Overview](https://github.com/mithi/Fusion-UKF-CPP/blob/master/images/FusionUKF_overview_1.png)

###  And here's an overview of what the instantiated classes are doing
![UKF Algorithm Overview 2](https://github.com/mithi/Fusion-UKF-CPP/blob/master/images/FusionUKF_overview_2.png)

# [🐳](https://mithi.github.io/deep-blueberry) [☕️](https://ko-fi.com/minimithi) [🧧](https://www.paypal.me/minimithi) 
