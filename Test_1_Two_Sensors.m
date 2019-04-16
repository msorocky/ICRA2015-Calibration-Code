% this script generates the first set of results in the paper
% It calculates the transformation between the velodyne and camera 0 for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables
clear;
clc;
%data range (start excluded as not all sensors running)
range = 10:1000;

%number of scans to use
scansRange = 10:50:800;
scansRange1 = 10:50:800;

%number of times to perform test
reps = 1;

%number of bootstrap iterations to perform
bootNum = 100;

%% setup folders
addAll();

%% clear previous data
tformIdx = 1;
clear tform;
clear tformVar;
clear sensorType;
clear sensorData;

%% process velodyne
load('zeusVelData.mat');             % data obtained from genKittiVel/genKittiVel.m
sensorData{tformIdx,1} = velData;    % This would be line 2 in the paper's Algorithm 1 
tformIdx = tformIdx + 1;

%% process nav
load('zeusNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

%% process cameras
% load('kittiCam1Data.mat');
% sensorData{tformIdx,1} = cam1Data;
% tformIdx = tformIdx + 1;

% load('kittiCam2Data.mat');
% sensorData{tformIdx,1} = cam2Data;
% tformIdx = tformIdx + 1;
% 
% load('kittiCam3Data.mat');
% sensorData{tformIdx} = cam3Data;
% tformIdx = tformIdx + 1;
% 
% load('kittiCam4Data.mat');
% sensorData{tformIdx} = cam4Data;
% tformIdx = tformIdx + 1;

%% find transformations

for i = 1:length(sensorData)
    if(i > 1)
        % This interpolates transforms from sensor A to match times of
        % corresponding sensor B transform      
        % matchTforms(sensorA, sensorB, range, addError) -- range is the
        % timesteps you care about, addError bool for adding noise to each
        % transformation
        sensorData{i} = matchTforms(sensorData{i}, sensorData{1}, range, false);
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

save('matched_traj.mat', 'sensorData')

RErr = zeros(reps,3,size(scansRange(:),1));
TErr = zeros(reps,3,size(scansRange(:),1));

RVar = zeros(reps,3,size(scansRange(:),1));
TVar = zeros(reps,3,size(scansRange(:),1));

RErrEqual = zeros(reps,3,size(scansRange(:),1));
TErrEqual = zeros(reps,3,size(scansRange(:),1));

for w = 1:reps
    for s = 1:size(scansRange(:),1)
        %get random contiguous scans to use
        % scansRange = [10 20 30 40 50] (defined at top of file)
        % randTforms(sensorData, n) returns n sequential transforms and
        % their variances, randomly from sensorData 
        % I.e. at each repetition, we'd have n transforms to use
        sData = randTforms(sensorData, scansRange(s));

        %Create equal weighted variance
        sDataE = sData;
        for i = 1:size(sData,1)
            sDataE{i}.T_Cov_Skm1_Sk = ones(size(sData{1}.T_Cov_Skm1_Sk));
        end

        % Give a coarse estimate of R and T using sensor data sDataE
        % (weighting variances equally) - lines 5, 8 in Algorithm 1
        rotVec = roughR(sDataE); 

        tranVec = roughT_new(sDataE, rotVec);

        %write out results
        RErrEqual(w,:,s) = rotVec(2,:);
        TErrEqual(w,:,s) = tranVec(2,:);

        %find rotation, now using variances previously obtained from sensor readings
        rotVec = roughR(sData);

        sData = findInR(sData, rotVec);
        % Refines initial guess of rotVec - this would be line 7 of
        % Algorithm 1
        rotVec = optR(sData, rotVec); 

        %find translation, now using variances obtained from sensor readings
        tranVec = roughT_new(sData, rotVec);
%         sData = findInT(sData, tranVec, rotVec);
        
        % Refines initial guess of tranVec - this would be line 7 of
        % Algorithm 1
%         tranVec = optT(sData, tranVec, rotVec);

        %bootstrap - line 10 in Algorithm 1
%       [tranVar, rotVar] = bootTform(sData, tranVec, rotVec, bootNum);

        %write out results
        RErr(w,:,s) = rotVec(2,:);
        TErr(w,:,s) = tranVec(2,:);
%         RVar(w,:,s) = rotVar(2,:);
%         TVar(w,:,s) = tranVar(2,:);

        fprintf('R = [% 1.3f,% 1.3f,% 1.3f], T = [% 3.2f,% 3.2f,% 3.2f] using %4i scans, iteration = %i\n',rotVec(2,1),rotVec(2,2),rotVec(2,3),tranVec(2,1),tranVec(2,2),tranVec(2,3),scansRange(s),w);


        save('Test_1_Res.mat', 'RErr', 'TErr', 'RVar', 'TVar', 'RErrEqual', 'TErrEqual','scansRange','scansRange1');

   

    end
end
        fprintf('Finish computing\n');
