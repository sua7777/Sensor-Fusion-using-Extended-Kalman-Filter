# Sensor-Fusion-using-Extended-Kalman-Filter

### Project aim: 
#### To estimate the position, velocity and orientation of a moving vehicle based on lidar-like measurements to known landmarks/features, GPS measurements and gyroscope measurements.

Process Model:
![image](https://user-images.githubusercontent.com/88794920/181086263-45f4b8d4-35d5-4b6a-871c-c15dea0550da.png)
  1. It has been assumed that the vehicle can travel at a constant speed in the 2D, X, Y plane in the direction it is facing,
  2. Inputs are turn rate of the vehicle from a gyroscope and assume that acceleration is a random variable.
  3. Sensor data fusion is done to fuse in a different sensor models.
  4. We will assume that we get position measurements of the vehicle location, via (GPS) sensor. [Sensor Model 1]
  5. Assume we get a number of range and relative bearing measurements to known landmark locations (i.e. LIDAR) [ Sensor Model 2]

Simulation Configurations:
The simulations are run for different configurations as follows-----------
  Profile 1 (Constant Speed/Heading, Zero Initial Conditions)
  Profile 2 (No-Zero Initial Conditions)
  Profile 3 (Constant Speed, Changing Headings)
  Profile 4 (Changing Speed, Changing Headings)
  Profile 5 (Profile 1 + LIDAR)
  Profile 6 (Profile 2 + LIDAR)
  Profile 7 (Profile 3 + LIDAR)
  Profile 8 (Profile 4 + LIDAR)

EKF Prediction Step:

Step 1 (Setup)
 > Open the c++ file “kalmanfilter.cpp” .
 > Compile the run the simulation as is, using profile 3. See that the car starts at the origin (0,0).
 > and moves at 5 m/s while performing a series of turns.
  The filter will start to run (but using GPS only and without an process model)
  
  ![image](https://user-images.githubusercontent.com/88794920/181087717-08de6269-876f-49dd-9fd5-ac8b90ee8b84.png)
  
  ![image](https://user-images.githubusercontent.com/88794920/181087776-5c0ed631-da0e-44d8-b3d3-51917406060a.png)
  
  ![image](https://user-images.githubusercontent.com/88794920/181087934-49029f0f-3f93-4676-b90f-6b1566a2f08b.png)

  Step 2 (Implement the Prediction Step (Process Model))
  > Code written in predictionStep() function.
  > Use the time step and gyroscope input. Assume zero acceleration.
  > Normalise the heading angle to +/- PI after prediction

  ![image](https://user-images.githubusercontent.com/88794920/181088123-6678a03f-eb13-44b0-9431-34805db6a2c9.png)
  
  Step 3 (Implement the Prediction Step (Covariance))
  > Code written in predictionStep() function
  > Calculate the Jacobian State matrix (F matrix)
  > Define the Q matrix for gyroscope noise and acceleration noise (Assume zero positional noise)
  > Implement the covariance prediction step.

  ![image](https://user-images.githubusercontent.com/88794920/181088341-843723e2-45b0-4e3a-b0fc-091b95ccf5b9.png)
  
  ![image](https://user-images.githubusercontent.com/88794920/181088386-dba1d80c-7af1-4495-858e-bc6e8bc48c5b.png)
  
 The inclusion of the gyroscope allows the filter to respond quicker to a directional changes, such as the car turning as the filter now has a concept 
 of direction/heading. (This cannot be done in the linear field due to the non-linear process model.)
 
 The EKF is sensitive to Initial Conditions (State & Covariance)
  1. EKF does not guarantee to converge (unlike LKF)
  2. EKF can diverge if the estimated state is too far away from the true state.
    (This is because we're using the linear approximation of the nonlinear system.)
  3. So we want to initialize it with the FULL state and covariance from measurement data.
    We can't just assume zero state with large uncertainty and expect it to converge.

 EKF Update step:
 The inclusion of the lidar measurements allows the filter to better estimate the state
 of the vehicle.
  
  1. Now we have information about the X and Y position and also heading of car.
  2. Lidar has faster measurement rate. (10 Hz compared to 1Hz GPS)
  3. More information (can have multiple measurements per time update, one measurements per detected landmark)
  
  Step 1 (Setup)
  handleLidarMeasurements() is called after the prediction step whenever LIDAR
  measurements have been made by the sensor. This function then sequentially calls the
  handleLidarMeasurement() function which is the main function to fuse
  the Lidar measurements one at a time.

  Step 2 (Implement the Lidar Measurement Model)
  Code written in the function handleLidarMeasurement()
  
  ![image](https://user-images.githubusercontent.com/88794920/181089080-a17d31a6-5e8c-4c08-8fac-a39d3c015289.png)

  Step 3 (Implement the Innovation Calculations)
  
  Code written in the function handleLidarMeasurement()
  Calculate the Measurement Innovation (The angle innovation is normalised).
    ![image](https://user-images.githubusercontent.com/88794920/181089212-2c058f78-21b4-47ac-95e3-bb94ac51cf6a.png)

  Calculate the Measurement Jacobian Matrix (H matrix)
  Calculate the Measurement Innovation Covariance (S matrix)
  Assume the range and theta measurement std are LIDAR_RANGE_STD and LIDAR_THETA_STD
  
  ![image](https://user-images.githubusercontent.com/88794920/181089260-19344e95-007d-4201-80e3-b40695218020.png)
  
  ![image](https://user-images.githubusercontent.com/88794920/181089292-7b74acfb-167d-4de1-9d0d-a0b08c21ba14.png)

  ![image](https://user-images.githubusercontent.com/88794920/181089325-905ca5f8-5a91-4be0-9c82-56e639e76460.png)

Step 4 (Implement the Kalman Filter Update Step Equations)
  
  ![image](https://user-images.githubusercontent.com/88794920/181089388-439f7ed4-f925-4c23-b938-e0e285c2de7e.png)
  
  ![image](https://user-images.githubusercontent.com/88794920/181089438-369b066e-042d-45b4-a90c-b1f5b38bf551.png)

  EKF Conclusion:
  1.Jacobian transformation is a linear approximation of the non-linear transformation (It works well if the system is approximately linear at linearisation  point).
  2.We can also see that large uncertainties can lead to more linearisation error effect.
  3.We can see that the linear approximation is most accurate in the linearization point.
  4.The linear approximation of uncertainty that uses the Jacobean tends to underestimate the error (leading to overconfident estimates)

  
Limitations:

1. Large estimation error occurs if....
    we have wrong/incomplete initial conditions.
    bad/faulty data or measurements.
    we miss data so we get a lack of information for a long period of time then the filter can actually start to drift.
    bad model assumptions.
    lack of updates (Drift)

2. So large estimation errors can cause the EKF to diverge when the linear approximation assumptions are invalid for the current conditions & we can't tell if this is happening (i.e. you would need to know the true state to know the estimation error has increased.

3. So we need to test all the assumptions and operating points to make sure that the filter stays stable and it's giving us good results.

4.inertial navigation systems use nonlinear equations and are typically implemented using the extended common filter. But in-general it works because the nonlinear navigation equations can be approximated quite well with linear approximations if we have some idea about what the current state is.

Step 5 (You can quickly compare to the LKF Solution for the Error
Statistics)

![image](https://user-images.githubusercontent.com/88794920/181089993-09965e14-b86a-4914-88e7-22d631b0f589.png)












