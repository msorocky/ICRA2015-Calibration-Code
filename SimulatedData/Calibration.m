% This script runs the entire calibration process. It imports and processes
% the data, runs the algorith, and then analyzes and plots the data.

% First, we need to select the type of data we are going to use. We can
% either use our generated simulated data, or our real data from the rosbag
% that Keenan provided.

real_data = false;

if real_data
    
    
else
    % Generate a random smooth trajectory from simulated data
    Test()    
end

% Next, we need to process the data
DataPro;

% Next, we need to run the calibration code
Veldy_to_IMU;

% Finally, display the results of our data
Plot_1_Two_Sensors_sim;
