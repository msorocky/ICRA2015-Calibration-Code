% Projection of trajectories
clear;
close all;

% Load the trajectory data
load('zeusNavData.mat');
load('zeusVelData.mat');

% Rotate the data
C = [0, sin(-pi/2), 0; -sin(-pi/2), 0, 0; 0, 0, 1];
t = [0;0;0];
T = [[C,t]; 0 0 0 1];

% Augment the velodyne data
velTraj = velData.T_S1_Sk;
vel_x_y_z = zeros(size(velTraj,1),3);
vel_th1 = zeros(size(velTraj,1),3);
vel_th2 = zeros(size(velTraj,1),3);
for i = 1:size(velTraj,1)
    th1 = velTraj(i,4);
    th2 = velTraj(i,5);
    th3 = velTraj(i,6);
    t_temp = [velTraj(i,1); velTraj(i,2); velTraj(i,3)];
    c_temp = [cos(th2)*cos(th3), cos(th1)*sin(th3)+sin(th1)*sin(th2)*cos(th3), sin(th1)*sin(th3)-cos(th1)*sin(th2)*cos(th3);...
        -cos(th2)*sin(th3), cos(th1)*cos(th3)-sin(th1)*sin(th2)*sin(th3), sin(th1)*cos(th3)+cos(th1)*sin(th2)*sin(th3);...
        sin(th2), -sin(th1)*cos(th2), cos(th1)*cos(th2)];
    velTraj_pose = [[c_temp, t_temp]; 0 0 0 1];
    %T(
    velTraj_pose = T*velTraj_pose;
    vel_x_y_z(i,:) = velTraj_pose(1:3,4);
    vel_th1(i,:) = [velTraj_pose(2,3), velTraj_pose(3,1), velTraj_pose(1,2)];
    vel_th2(i,:) = rot2vec(velTraj_pose(1:3,1:3));
    velTraj_pose = [];
end

% Organize the data
navTraj = navData.T_S1_Sk(1:11:end,:);
velTraj = velData.T_S1_Sk;
x_nav = -navTraj(:,1);
y_nav = navTraj(:,2);
z_nav = navTraj(:,3);
x_vel = -velTraj(:,1);
y_vel = velTraj(:,2);
z_vel = velTraj(:,3);
vel_x_y_z(:,1) = -vel_x_y_z(:,1);

% Create the figure
figure;
plot3(x_nav,y_nav,z_nav);
hold on;
plot3(vel_x_y_z(:,1),vel_x_y_z(:,2),vel_x_y_z(:,3));
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


