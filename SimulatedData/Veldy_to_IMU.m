clear
clc
% data range (start excluded as not all sensors running)
% Here is the problem, the ValData only has 135 scan (Nav has 1408 scans) 
% (it should have at least 200 scans)
range = 10:900;

% number of scans to use     
scansRange = 20:40:800;   % 10:10:50 

%number of times to perform test
reps = 100;

%number of bootstrap iterations to perform
bootNum = 50;

% setup folders
simulation=true;

% clear previous data
tformIdx = 1;
clear tform;
clear tformVar;
clear sensorType;
clear sensorData;

% process velodyne
load('SimulatedData/Sim_Data_noise1.mat'); % data obtained from genKittiVel/genKittiVel.m
sensorData{tformIdx,1} = velData; % This would be line 2 in the paper's Algorithm 1 
tformIdx = tformIdx + 1;

% process nav
% load('Kitti_0930/kittiNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

% find transformations      

RErr = zeros(reps,3,size(scansRange(:),1));
TErr = zeros(reps,3,size(scansRange(:),1));

RVar = zeros(reps,3,size(scansRange(:),1));
TVar = zeros(reps,3,size(scansRange(:),1));

RErrEqual = zeros(reps,3,size(scansRange(:),1));
TErrEqual = zeros(reps,3,size(scansRange(:),1));

for w = 1:reps
    for s = 1:size(scansRange(:),1)  % 20
        %get random contiguous scans to use
        % scansRange = [10 20 30 40 50] (defined at top of file)
        
        % randTforms(sensorData, n) returns n sequential transforms and
        % their variances, randomly from sensorData 
        % I.e. at each repetition, we'd have n transforms to use
        sData = randTforms_sim(sensorData, scansRange(s));

        %Create equal weighted variance
        sDataE = sData;
        for i = 1:size(sData,1)
            sDataE{i}.T_Cov_Skm1_Sk = ones(size(sData{1}.T_Cov_Skm1_Sk));
        end

        % Give a coarse estimate of R and t using sensor data sDataE
        % (weighting variances equally) - lines 5, 8 in Algorithm 1
        rotVec = roughR(sDataE); 
        tranVec = roughT(sDataE, rotVec);

        %write out results. REeeEqual and TErrEqual are the results of the
        %weighted least square(old method)
        
        RErrEqual(w,:,s) = rotVec(2,:);
        TErrEqual(w,:,s) = tranVec(2,:);

        %find rotation, now using variances previously obtained from sensor readings
        rotVec = roughR(sData);
        sData = findInR(sData, rotVec);
        % Refines initial guess of rotVec - this would be line 7 of
        % Algorithm 1
        rotVec = optR(sData, rotVec); 
        
        %find translation, now using variances obtained from sensor readings
        tranVec = roughT(sData, rotVec);
        sData = findInT(sData, tranVec, rotVec);
        
        % Refines initial guess of tranVec - this would be line 7 of
        % Algorithm 1
        tranVec = optT(sData, tranVec, rotVec);

        %bootstrap - line 10 in Algorithm 1
        % [tranVar, rotVar] = bootTform(sData, tranVec, rotVec, bootNum);   %

        %write out results
        RErr(w,:,s) = rotVec(2,:);
        TErr(w,:,s) = tranVec(2,:);
  %     RVar(w,:,s) = rotVar(2,:);                                       %
  %     TVar(w,:,s) = tranVar(2,:);                                      %

  %     fprintf('R = [% 1.3f,% 1.3f,% 1.3f], T = [% 3.2f,% 3.2f,% 3.2f] using %4i scans, iteration = %i\n',...
  %             rotVec(2,1),rotVec(2,2),rotVec(2,3),tranVec(2,1),tranVec(2,2),tranVec(2,3),scansRange(s),w);

        save('SimulatedData/Sim_Results_noise1.mat', 'RErr', 'TErr', 'RVar', 'TVar', 'RErrEqual', 'TErrEqual','Groundtruth');
    end
end
        fprintf('Finished computing\n')














