% this script generates the first set of results in the paper
% It calculates the transformation between the velodyne and camera 0 for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

% user set variables
clear;
clc;
%data range (start excluded as not all sensors running)
range = 10:1600;

%number of scans to use
scansRange = 10:100:1500;
scansRange1 = 10:100:1500;

%number of times to perform test
reps = 1;

%number of bootstrap iterations to perform
bootNum = 100;

% setup folders
addAll();

% clear previous data
tformIdx = 1;
clear tform;
clear tformVar;
clear sensorType;
clear sensorData;

% process velodyne
load('zeusVelData.mat');          % data obtained from genKittiVel/genKittiVel.m
sensorData{tformIdx,1} = velData; % This would be line 2 in the paper's Algorithm 1 
tformIdx = tformIdx + 1;

% process nav
load('zeusNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;


% sensorData{1}.T_Skm1_Sk(:,3)=sensorData{1}.T_Skm1_Sk(:,3)*20;
% sensorData{2}.T_Skm1_Sk(:,3)=sensorData{2}.T_Skm1_Sk(:,3)*20;
% find transformations
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

RErr = zeros(reps,3,size(scansRange(:),1));
TErr = zeros(reps,3,size(scansRange(:),1));

RVar = zeros(reps,3,size(scansRange(:),1));
TVar = zeros(reps,3,size(scansRange(:),1));

RErrEqual = zeros(reps,3,size(scansRange(:),1));
TErrEqual = zeros(reps,3,size(scansRange(:),1));
%TODO: Initialize SData_T
sData_T = cell(2,1,size(scansRange(:),1),reps);
for w = 1:reps
    for s = 1:size(scansRange(:),1)
        % get random contiguous scans to use
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
        rotVec_E = roughR(sDataE);
        
        r_Vel2IMU_E = vec2rot(rotVec_E(2,:)');
        r_IMU2Vel_E = r_Vel2IMU_E';
        rotVec_E = rot2vec(r_IMU2Vel_E);
        rotVec_E = [[0 0 0];rotVec_E'];
        
        %write out results
        RErrEqual(w,:,s) = rotVec_E(2,:);

        %find rotation, now using variances previously obtained from sensor readings
        rotVec = roughR(sData);
        sData = findInR(sData, rotVec);
        % Refines initial guess of rotVec - this would be line 7 of
        % Algorithm 1
        rotVec_F = optR(sData, rotVec);
        
        r_Vel2IMU_F = vec2rot(rotVec_F(2,:)');
        r_IMU2Vel_F = r_Vel2IMU_F';
        rotVec = rot2vec(r_IMU2Vel_F);
        rotVec = [[0 0 0];rotVec'];
        
        tranVec = roughT_new(sDataE, rotVec_F);
        TErrEqual(w,:,s) = tranVec(2,:);
        
        %bootstrap - line 10 in Algorithm 1
        [tranVar, rotVar, weight] = bootTform(sData, tranVec, rotVec_F, bootNum);
%       TODO: Store SData into sData_T so the exact same trajectory will be used for Translation
        sData_T(:,:,s,w) =  sData;
        
        
        RErr(w,:,s) = rotVec(2,:);
        RVar(w,:,s) = rotVar(2,:);
        
        fprintf('R = [% 1.3f,% 1.3f,% 1.3f], using %4i scans, iteration = %i\n',rotVec(2,1),rotVec(2,2),rotVec(2,3),scansRange(s),w);
        
        save('repeat_1.mat', 'RErr', 'RVar', 'RErrEqual', 'TErrEqual', 'scansRange');
    end
end

 
for w = 1:reps
    for s = 1:size(scansRange1(:),1)
%         sData = randTforms(sensorData, scansRange1(s));
%         tranVec = roughT(sDataE, rotVec);
%         TErrEqual(w,:,s) = tranVec(2,:);

%       TODO: For each step we need the exact trajectory of Rotation SData_T to
%       be used 

        sData = sData_T(:,:,s,w);
        
%         r_Vel2IMU = vec2rot(rotVec(2,:)');
%         r_IMU2Vel = r_Vel2IMU';
%         rotVec = rot2vec(r_IMU2Vel);
        %find translation, now using variances obtained from sensor readings
        tranVec = roughT_new(sData, rotVec_F);
         sData = findInT(sData, tranVec, rotVec_F);
        
        % Refines initial guess of tranVec - this would be line 7 of
        % Algorithm 1
         tranVec = optT(sData, tranVec, rotVec_F);

        %bootstrap - line 10 in Algorithm 1
        [tranVar, rotVar, weight] = bootTform(sData, tranVec, rotVec_F, bootNum);

        %write out results
%         RErr(w,:,s) = rotVec(2,:);
        TErr(w,:,s) = tranVec(2,:);
%         RVar(w,:,s) = rotVar(2,:);
        TVar(w,:,s) = tranVar(2,:);

        fprintf('T = [% 3.2f,% 3.2f,% 3.2f] using %4i scans, iteration = %i\n',tranVec(2,1),tranVec(2,2),tranVec(2,3),scansRange(s),w);

        save('repeat_1.mat', 'TErr', 'TVar','scansRange1', '-append');
    end
end
        fprintf('Finish computing\n');
