# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

UKF uses Sigma Points. Sigma Points allows us to do non linear approximation better than a linearization does for EKF. It's able to work with more sophisticated process model that is able to estimate the turn rate of the vehicle. We are using a Constant Turn Rate and Velocity Magnitude Model. 

UKF Workflow consists of the following steps:

##### I. Prediction
1. Generate Sigma Points
2. Predict Sigma Points
3. Predict Mean and Covariance

##### II. Update
1. Predict Measurement
2. Update State 

![Alt text](images/workflow.png?raw=true "Title")

##### NIS Calculation
Normalized Innovation Squared is used to check the consistency of the filter and is given by the below formula. 
![Alt text](images/nis.png?raw=true "Title")


This project involves a Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


![Alt text](images/data-ss.png?raw=true "Title")

Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).


Simulator Final State for Dataset1

![Alt text](images/dataset1-output.png?raw=true "Title")

Simulator Final State for Dataset2

![Alt text](images/dataset2-output.png?raw=true "Title")


---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o from the simulator.

## Generating Additional Data for Experiments

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Real World Object Tracking

Castro St., Mountain View, California 

[![Real World Object Tracking](https://img.youtube.com/vi/FMNJPX_sszU/0.jpg)](https://www.youtube.com/watch?v=FMNJPX_sszU "Everything Is AWESOME")
