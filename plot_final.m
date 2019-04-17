% Plot of final trajectories

clc
close all

load('matched_traj.mat', 'sensorData')

%compute_calibration;
calib = [0.000, 0.046, 0.399, -0.000, -0.000, -1.571];

velodyne_traj = sensorData{1}.T_S1_Sk(:,:);
gps_imu_traj = sensorData{2}.T_S1_Sk(:,:);

t = sensorData{1}.time;
t = t - t(1); % Make time (for plotting) relative to the start of the trajectory rather than in Unix time

N = size(t,1); % # timesteps

t_error = zeros(N, 3);
r_error = zeros(N, 3);

T_gi_aligned = zeros(size(gps_imu_traj));

for n = 1 : N
   
    % Convert 6x1 vectors into transformation matrices
    T_gi = vec2tran(gps_imu_traj(n,:)'); % GPS-IMU
    T_vel = vec2tran(velodyne_traj(n,:)'); % Velodyne
    T_calib = vec2tran((calib)'); % calibration
    
    % T_gi * calib should transform GPS-IMU frame into the Velodyne frame
    T_gi_aligned(n,:) = tran2vec(T_calib * T_gi);    
        
end

figure;
plot3(-velodyne_traj(:,1),velodyne_traj(:,2),velodyne_traj(:,3));
hold on;
plot3(-gps_imu_traj(:,1),gps_imu_traj(:,2),gps_imu_traj(:,3));
hold on;
plot3(-T_gi_aligned(:,1),T_gi_aligned(:,2),T_gi_aligned(:,3));
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
grid on;
legend('Velodyne Trajectory', 'IMU Trajectory', 'Aligned IMU Trajectory');