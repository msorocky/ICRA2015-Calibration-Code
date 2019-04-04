function [ velData ] = genKittiVel( kittiPath, plotVel, range )
%GENLADYBUG Generates transforms using icp and the velodyne

%setup

%setup help info
velData.help = ['velData stores the following information:\n'...
'folder- the folder containing the scans used\n'...
'files- the name of the velodyne files used to find the transforms\n'...
'help- this information...'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'];

velData.folder = [kittiPath 'velodyne_points/data/'];

velData.files = dir([velData.folder,'*.txt']);

if(isempty(range))
    range = 1:size(velData.files(:));
end

velData.files = velData.files(range);
velData.T_S1_Sk = zeros(size(velData.files(:),1),6);
velData.T_S1_Sk(1,:) = tran2vec(eye(4));
velData.T_Skm1_Sk = zeros(size(velData.files(:),1),6);
velData.T_Skm1_Sk(1,:) = tran2vec(eye(4));
velData.T_Cov_Skm1_Sk = zeros(size(velData.files(:),1),6);
velData.T_Cov_Skm1_Sk(1,:) = inf(1,6);

[velData.time, velData.timeStart, velData.timeStop] = ReadKittiTimestamps([velData.folder '../']);
velData.time = velData.time(range);
velData.timeStart = velData.timeStart(range);
velData.timeStop = velData.timeStop(range);

velData.type = 'lidar';


if(plotVel)
    figure;
    axis equal;
    hold on;
end

%setup loop
%vel = ReadKittiVelDataSingle( [velData.folder velData.files(1).name] );
vel = dlmread([velData.folder velData.files(1).name],' ');

%find transform for each velodyne scan


