function [t_error, r_error] = compute_alignment_error(calib)
% Computes alignment error between a GPS-IMU trajectory and a Velodyne
% trajectory

% @param calib 1x6 vector: [x y z r p y], representing the pose
% of the Velodyne in the GPS/IMU frame

% @return t_error Nx3 vector of translation errors
% @return r_error Nx3 vector of rotation errors

clc
close all

load('matched_traj.mat', 'sensorData')

velodyne_traj = sensorData{1}.T_S1_Sk(:,:);
gps_imu_traj = sensorData{2}.T_S1_Sk(:,:);

t = sensorData{1}.time;
t = t - t(1); % Make time (for plotting) relative to the start of the trajectory rather than in Unix time

N = size(t,1); % # timesteps

t_error = zeros(N, 3);
r_error = zeros(N, 3);

for n = 1 : N
   
    % Convert 6x1 vectors into transformation matrices
    T_gi = vec2tran(gps_imu_traj(n,:)'); % GPS-IMU
    T_vel = vec2tran(velodyne_traj(n,:)'); % Velodyne
    T_calib = vec2tran(calib'); % calibration
    
    % T_gi * calib should transform GPS-IMU frame into the Velodyne frame
    T_gi_aligned = T_gi * T_calib;
    
    % Compute the position error
    t_error(n, :) = T_gi_aligned(1:3, 4) - T_vel(1:3, 4);
    
    % Skew symmetric matrix for the rotation error; ideally 0
    ss_r_error = eye(3) - T_gi_aligned(1:3, 1:3) * T_vel(1:3, 1:3)';
    
    r_error(n, 1) = ss_r_error(3,2);
    r_error(n, 2) = -1 * ss_r_error(3,1);
    r_error(n, 3) = ss_r_error(2,1);
    
end

% Plots

subplot(6,2,1)
plot(t, t_error(:, 1))
xlabel('t [s]')
ylabel('e_x [m]')

subplot(6,2,3)
plot(t, t_error(:, 2))
xlabel('t [s]')
ylabel('e_y [m]')

subplot(6,2,5)
plot(t, t_error(:, 3))
xlabel('t [s]')
ylabel('e_z [m]')

subplot(6,2,2)
plot(t, r_error(:, 1))
xlabel('t [s]')
ylabel('e_{\delta\theta_x} [rad]')

subplot(6,2,4)
plot(t, r_error(:, 2))
xlabel('t [s]')
ylabel('e_{\delta\theta_y} [rad]')

subplot(6,2,6)
plot(t, r_error(:, 3))
xlabel('t [s]')
ylabel('e_{\delta\theta_z} [rad]')
