# **Extended Kalman Filter Project** 

## Writeup


---

[//]: # (Image References)

[image1]: ./images/algorithm-general-flow.png "Algorithm General Flow"
[image2]: ./examples/grayscale.jpg "Grayscaling"


**Build a Kalman Filter Project**

The goal of this project is to build an Kalman filter algorithm in CPP that it capable of predicting the car position and speed by compining the readings from laser, and radar packages using Kalman filter algorithm.

The steps of this project are summarized in [this image:][image1]
![alt text][image1]




## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Compiling

#### 1. Your code should compile.

I used the [visual studio project for term 2](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio) mentioned in [Environment setup for windows lesson](https://classroom.udacity.com/nanodegrees/nd013-ent/parts/f114da49-70ce-4ebe-b88c-e0ab576aed25/modules/75324158-265f-4b73-bedc-2c30a7ac4db6/lessons/5b02bb87-caf5-43c0-b45a-599792af53a1/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).
You can find my project inside the `CarND-Term2-ide-profile-VisualStudio/VisualStudio` folder on the repo.
I used visual studio 2015.
You can open the project solution `CarND-Extended-Kalman-Filter-Project.sln` if you have visual studio 2015 installed.

The source files are inside `src` folder on the repo.

### Accuracy

#### 1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

My algorithm outputs lower values for both dataset1, and dataset2 in the simulator. For dataset 1 my RMSE outputs are:

| RMSE variable        		|     Value	        					| 
|:---------------------:|:---------------------------------------------:| 
| X         		| 0.0966   							| 
| Y      	| 0.0852 	|
| VX					|	0.4156											|
| VY	      	| 0.4325				|

For dataset 2 my RMSE outputs are:

| RMSE variable        		|     Value	        					| 
|:---------------------:|:---------------------------------------------:| 
| X         		| 0.0727  							| 
| Y      	| 0.0969 	|
| VX					|	0.4897											|
| VY	      	| 0.5083				|



### Follows the Correct Algorithm

#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

I followed the same steps given by the Kalman filter algorithm shown in [algorithm general flow image:][image1].


#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.

I initialized the kalman filter variables:
 * Measurement covariance matrix (R) matrix for laser `R_laser_`, and radar `R_radar_`.
 * Measurements matrix (H) matrix for laser `H_laser`, and radar `H_j`.
 * F matrix `F_`.
 * P matrix `P_`.
 * Noise variables `noise_ax`, `noise_ay` with value 9 as required in the project.
 * State matrix (x) that contains 4 values [x, y, velocity for x, velocity for y] is initialized by all 1s in the beginning.


#### 3. Your Kalman Filter algorithm first predicts then updates.

The main function of the algorithm can be found in function `ProcessMeasurement` which exists in file `FusionEKF.cpp`.
After receiving the first measurement, the algorithm computes the time difference between the last reading and the current reading.
It updated the `F_`, and `Q_` , predicts the new states, and finally it updates the `x_`, and `P_` matrices.

I wrote a function `isReasonableNewX` that checks the value of the new state before updating the current state `x_` with this new state.
The checks are done on x, y values so that the difference between the old_x, and the new_x has to be less than 0.3, and the same for the difference between new_y, old_y.
This is a recovery mechanism to detect any false readings, and to help my vehicle to be always on the track.
This function is used when updating both laser, and radar measurement, and initialy, it neglect the first four readings measurements.

#### 4. Your Kalman Filter can handle radar and lidar measurements.

As shown in function `ProcessMeasurement` in file `fusionEKF.cpp`, the predict function is called in either types of packages.
There are 2 update functions, one for the laser, and the other one for the radar.
These 2 functions are implemented inside `kalman_filter.cpp`.
 

### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.

I tried to avoid any calculation more than once as much as I can by following these techniques:
 * Store the value of any calculation that is used more than once in a variable.
 * Try to get any calculations out of loops if possible.
 * Avoid unnecessary checks.

