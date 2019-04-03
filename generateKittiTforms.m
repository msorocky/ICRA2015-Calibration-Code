% this script generates all the required transforms for the kitti
% dataset

%% user set variables

%path to data

kittiPath = '/home/mike/velodyne_ws/';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%Setup folders
addAll();

%% Run motion estimation

%do things in parrallel to save time
parfor i = 1:6
    switch i
        case 1
            kittiVelData = genKittiVel2(kittiPath, plotTforms, []);
            parsave('kittiVelData.mat', kittiVelData, 'velData');
%         case 2
%             kittiCamData = genKittiCam(kittiPath, plotTforms, [], 2);
%             parsave('kittiCam2Data.mat', kittiCamData, 'cam2Data');
%         case 3
%             kittiCamData = genKittiCam(kittiPath, plotTforms, [], 3);
%             parsave('kittiCam3Data.mat', kittiCamData, 'cam3Data');
%         case 4
%             kittiCamData = genKittiCam(kittiPath, plotTforms, [], 4);
%             parsave('kittiCam4Data.mat', kittiCamData, 'cam4Data');
%         case 5
%             kittiCamData = genKittiCam(kittiPath, plotTforms, [], 1);
%             parsave('kittiCam1Data.mat', kittiCamData, 'cam1Data');
        otherwise
    end
end
    
delete(gcp);