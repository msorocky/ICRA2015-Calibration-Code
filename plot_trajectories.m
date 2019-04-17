% Projection of trajectories
clear;
close all;

compute_calibration;

% Load the trajectory data
% load('zeusNavData.mat');
% load('zeusVelData.mat');

tformIdx = 1;

%% process velodyne
load('zeusVelData.mat');          % data obtained from genKittiVel/genKittiVel.m
sensorData{tformIdx,1} = velData; % This would be line 2 in the paper's Algorithm 1 
tformIdx = tformIdx + 1;

range = 1:size(velData.T_S1_Sk,1);

%% process nav
load('zeusNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

%% Interpolate the trajectories

for i = 1:length(sensorData)
    if(i > 1)
        % This interpolates transforms from sensor A to match times of
        % corresponding sensor B transform      
        % matchTforms(sensorA, sensorB, range, addError) -- range is the
        % timesteps you care about, addError bool for adding noise to each
        % transformation
        sensorData{i} = matchTforms(sensorData{i}, sensorData{1},range, false);
    else
        % T_Skm1_Sk is the transformation from the frame of the sensor at 
        % timestep k-1 to its frame at timestep k; T_Cov_Skm1_Sk is its
        % associated covariance
        % T_S1_Sk is the transformation from t = 1 to t = k
        sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(range,:);
        sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(range,:);
        sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(range,:);
        sensorData{i}.time = sensorData{i}.time(range,:);
        sensorData{i}.files = sensorData{i}.files(range,:);
    end
end

% Rotate the data (just assuming a 90 degree rotation for now)
C = [0, sin(-pi/2), 0; -sin(-pi/2), 0, 0; 0, 0, 1];
t = [0;0;0];
T = [[C,t]; 0 0 0 1];

% Augment the velodyne data
velTraj = sensorData{1}.T_S1_Sk;
vel_rotated = zeros(size(velTraj,1),3);
vel_th1 = zeros(size(velTraj,1),3);
vel_th2 = zeros(size(velTraj,1),3);

for i = 1:size(velTraj,1)
    velTraj_pose = vec2tran(velTraj(i,:)');
    velTraj_pose = T*velTraj_pose;
    vel_rotated(i,:) = velTraj_pose(1:3,4);
    vel_th1(i,:) = [velTraj_pose(2,3), velTraj_pose(3,1), velTraj_pose(1,2)];
    vel_th2(i,:) = rot2vec(velTraj_pose(1:3,1:3));
    velTraj_pose = [];
end

% Organize the data
navTraj = sensorData{2}.T_S1_Sk;
velTraj = sensorData{1}.T_S1_Sk;
x_nav = -navTraj(:,1);
y_nav = navTraj(:,2);
z_nav = navTraj(:,3);
x_vel = -velTraj(:,1);
y_vel = velTraj(:,2);
z_vel = velTraj(:,3);
vel_rotated(:,1) = -vel_rotated(:,1);

% Create the figure
figure;
plot3(x_nav,y_nav,z_nav);
hold on;
plot3(vel_rotated(:,1),vel_rotated(:,2),vel_rotated(:,3));
hold on;
plot3(x_vel,y_vel,z_vel);
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
grid on;
legend('IMU Trajectory', 'Lidar Trajectory (Rotated)', 'Lidar Trajectory (Original)');

% Create the figure
figure;
plot3(x_nav,y_nav,z_nav);
hold on;
plot3(vel_rotated(:,1),vel_rotated(:,2),vel_rotated(:,3));
hold on;
plot3(x_vel,y_vel,z_vel);
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
grid on;
legend('IMU Trajectory', 'Lidar Trajectory (Rotated)', 'Lidar Trajectory (Original)');

% figure;
% subplot(3,1,1);
% plot(vel_th1(:,1));
% hold on;
% plot(navTraj(:,4));
% hold on
% plot(velTraj(:,4));
% xlabel('Pose');
% ylabel('roll angle in rad');
% legend('vel aug','nav','vel original');
% 
% subplot(3,1,2);
% plot(vel_th1(:,2));
% hold on;
% plot(navTraj(:,5));
% hold on
% plot(velTraj(:,5));
% xlabel('Pose');
% ylabel('pitch angle in rad');
% legend('vel aug','nav','vel original');
% 
% subplot(3,1,3);
% plot(vel_th1(:,3));
% hold on;
% plot(navTraj(:,6));
% hold on
% plot(velTraj(:,6));
% xlabel('Pose');
% ylabel('yaw angle in rad');
% legend('vel aug','nav','vel original');
