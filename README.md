# Extended Kalman Filter Project Starter Code
----
This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

# Program Architecture
----

- The src folder has the source code of the project
- It has the following files:	
  1] [FusionEKF.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/FusionEKF.cpp)	
  2] [FusionEKF.h](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/FusionEKF.h)    
  3] [json.hpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/json.hpp) 	
  4] [kalman_filter.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/kalman_filter.cpp)	
  5] [kalman_filter.h](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/kalman_filter.h)	 
  6] [main.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/main.cpp) 	
  7] [measurement_package.h](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/measurement_package.h)	
  8] [tools.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/tools.cpp)   
  9] [tools.h](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/tools.h)
  
# Source Code File Description
----
1] [main.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/main.cpp):
   - This is where the code execution starts
   - This file Communicates with the Term 2 Simulator receiving data measurements
   - main.cpp reads in the sensor [data](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/data/obj_pose-laser-radar-synthetic-input.txt) which is a txt file  
   - After reading the sensor data the code figures out whether the sensor data is LIDAR or RADAR and accordingly set the the state matrix and sends a sensor measurement to FusionEKF.cpp
   - Calls `ProcessMeasurement()` function of class `FusionEKF` to process the data. The body `ProcessMeasurement()` is defined in [FusionEKF.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/FusionEKF.cpp) file 
   - Calls `CalculateRMSE()` function of class `Tools` to calculate RMSE. The body `CalculateRMSE()` is defined in [tools.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/tools.cpp) file 
   
2] [FusionEKF.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/FusionEKF.cpp)	
   - Initializes the kalman filter
   - Calls `Predict()` function of `KalmanFilter` class. The body `Predict()` is defined in [kalman_filter.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/kalman_filter.cpp) file
   - Calls `UpdateEKF()` function of `KalmanFilter` class if the sensor data is RADAR. The body `UpdateEKF()` is defined in [kalman_filter.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/kalman_filter.cpp) file. `UpdateEKF()` function updates the state matrix with extended kalman filter function by calcualting the Jacobian.   
   - Calls `Update()` function of `KalmanFilter` class if the sensor data is LASER. The body `UpdateEKF()` is defined in [kalman_filter.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/kalman_filter.cpp) file. `Update()` function updates the state matrix with simple kalman filter function (No need to calcualte the Jacobian).
   
3] [kalman_filter.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/kalman_filter.cpp)
   - Defines the `Predict()` function of `KalmanFilter` class
   - Defines the `UpdateEKF()` function of `KalmanFilter` class
   - Defines the `Update()` function of `KalmanFilter` class
   - The above function implements the actual calcualtions of the kalman filter 

4] [tools.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/tools.cpp) 
   - Defines the function `CalculateRMSE()` function of class `Tools` to calculate RMSE 
   - Defines the function `CalculateJacobian()` function of class `Tools` to calculate Jacobian matrix


# Required Installation and Detials

- Download the Simulator [here](https://github.com/udacity/self-driving-car-sim/releases)
- [uWebSocketIO](https://github.com/uNetworking/uWebSockets) package is required so that main.cpp can communicate with the simulator
- [uWebSocketIO](https://github.com/uNetworking/uWebSockets) package facilitates the connection between the simulator and [main.cpp](https://github.com/jayshah19949596/Extended-Kalman-Filter/blob/master/src/main.cpp)
- uWebSocketIO installtion on Linux: From the project repository directory run the script `install-ubuntu.sh` which will do the installtion for you
- uWebSocketIO installtion on Mac: From the project repository directory run the script `install-mac.sh` which will do the installtion for you
- uWebSocketIO installtion on Windows: Follow the instruction on the [link](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10) to install bash on the Windows and with the bash run the script `install-ubuntu.sh` which will do the installtion for you

# Important Dependencies
----
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
  
  
# Basic Build Instructions
----
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


# Running the Simulator 
---

1] Open the Simulator after you run the `./ExtendedKF ` command   
2] Click on "SELECT" Option   
3] Click on "START" Option  


# INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


# OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x  
["estimate_y"] <= kalman filter estimated position y  
["rmse_x"]  
["rmse_y"]  
["rmse_vx"]  
["rmse_vy"]  
