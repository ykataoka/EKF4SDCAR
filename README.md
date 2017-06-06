# Extended Kalman Filter (Udacity Self-Driving Car Engineer Nanodegree Program)

In this project utilize a kalman filter to estimate the state of a
moving object of interest with noisy lidar and radar
measurements. Passing the project requires obtaining RMSE values that
are lower that the tolerance outlined in the project reburic.

This project involves the Term 2 Simulator which can be downloaded
[here](https://github.com/udacity/self-driving-car-sim/releases)


## Summary

The accuracy with the dataset 1 should results in 

    state    |	  rmse
:-----------:|:------------:
position_x   |    0.098
position_y   |    0.086
velocity_x   |    0.448
velocity_y   |    0.433



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`
5. Launch Simulator: Run simulator and get the result.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
