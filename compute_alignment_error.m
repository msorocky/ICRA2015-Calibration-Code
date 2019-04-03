function [t_error, r_error] = compute_alignment_error(gps_imu_traj, velodyne_traj, calib)
% Computes alignment error between a GPS-IMU trajectory and a Velodyne
% trajectory

% @param T_gps_imu matrix where rows are 6x1 vectors representing a GPS/IMU trajectory [xyz rpy]
% @param T_velodyne matrix where rows are 6x1 vectors representing a Velodyne trajectory [xyz rpy]
% @param calib 4x4 homogeneous transformation matrix representing the pose
% of the Velodyne in the GPS/IMU frame

% @return t_error Nx3 vector of translation errors
% @return r_error Nx3 vector of rotation errors

N = size(gps_imu_traj,1); % # timesteps

t_error = zeros(N, 3);
r_error = zeros(N, 3);

for n = 1 : N
   
    % Convert 6x1 vectors into transformation matrices
    T_gi = vec2tran(gps_imu_traj(n,:)); % GPS-IMU
    T_vel = vec2tran(velodyne_traj(n,:)); % Velodyne

    % T_gi * calib should transform GPS-IMU frame in the Velodyne frame
    T_gi_aligned = T_gi * calib;
    
    % Compute the position error
    t_error(n, :) = T_gi_aligned(1:3, 4) - T_vel(1:3, 4);
    
    % Skew symmetric matrix for the rotation error; ideally 0
    ss_r_error = eye(3) - T_gi_aligned(1:3, 1:3) * T_vel(1:3, 1:3)';
    
    r_error(n, 1) = ss_r_error(3,2);
    r_error(n, 2) = -1 * ss_r_error(3,1);
    r_error(n, 3) = ss_r_error(2,1);
    
end
