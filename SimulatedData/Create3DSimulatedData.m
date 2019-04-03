%
% Copyright � 2012, The Massachusetts Institute of Technology. All rights reserved. 
%
% THE LICENSOR EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS 
% SOFIWARE AND DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR ANY PARTICULAR PURPOSE, NON- INFRINGEMENT AND WARRANTIES OF 
% PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM COURSE OF 
% DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH 
% RESPECT TO THE USE OF THE SOFIWARE OR DOCUMENTATION. Under no circumstances 
% shall the Licensor be liable for incidental, special, indirect, direct or 
% consequential damages, or loss of profits, interruption of business, or 
% related expenses which may arise from use of Software or Documentation, 
% including but not limited to those resulting from defects in Software 
% and/or Documentation, or loss or inaccuracy of data of any kind. 
%
% This software is licensed under the "LIMITED RESEARCH LICENSE (SOURCE
% CODE)" as described in the included LICENSE.txt
%
% Please cite the paper below if you are using this software in your work:
% Brookshire, J.; Teller, S. Extrinsic Calibration from Per-Sensor Egomotion. 
%   Robotics: Science and Systems, 2012.
%
function run = Create3DSimulatedData(sigma_tra,sigma_rot,N,numberOfRuns,calib)

    if ( ~exist('calib', 'var') )
        [r,p,w]=quat_to_roll_pitch_yaw(angle_axis_to_quat(pi/5, [.4 .5 .6]));
        calib = [.02 .03 .04 r p w ];
    end
    
    % Expressing where sensor 2 is w.r.t. sensor 1.
    sensor2_expressedIn_sensor1 = GetHomoTransform(calib);
    
%     th = 0:0.05:pi;
%     r = 1*cos(3*th);
%     
%     x = 0:0.1:6.5;
%     y = x.^2;
%     z = sin(x)+cos(y);     
    
    % Parameters for the random trajectory
    N_interp = 300; % fidelity of the surface generated
    N_modes = ceil(N/10); % number of random sinusoids that are added to create surface
    rad = 30; % radius of circle
    amp_range = [0.5, 1];
    freq_range = [0.5, 1.5];
    
    % N - number of poses
    % rad - 
    % N_interp -
    % N_modes - 
    % amp_range - 
    % freq_range - 
    [x, y, z,dz_dx,dz_dy] = random_smooth_traj(N,rad, N_interp, N_modes, amp_range, freq_range);
 
    xaxis = [diff(x); diff(y); diff(z)];
    yaxis = zeros(size(xaxis));
    zaxis = zeros(size(xaxis));
    normal = zeros(size(xaxis));
    
%     dz_dx = cos(x);
%     dz_dy = -sin(y);

    for i = 1:length(x)-1
        normal(:,i) = cross( [1 0 dz_dx(i)], [0 1 dz_dy(i) ] );
        normal(:,i) = normal(:,i) / norm(normal(:,i));
        xaxis(:,i) = xaxis(:,i) /  norm(xaxis(:,i));        
        yaxis(:,i) = cross ( normal(:,i), xaxis(:,i) );
        yaxis(:,i) = yaxis(:,i) /  norm(yaxis(:,i));
        zaxis(:,i) = cross ( xaxis(:,i), yaxis(:,i) );
        zaxis(:,i) = zaxis(:,i) /  norm(zaxis(:,i));
    end
    
    figure(1);
    plot3(x,y,z);
    hold on;
%     quiver3(x,y,z, zaxis(1,:), zaxis(2,:), zaxis(3,:), 0);
%     quiver3(x,y,z, xaxis(1,:), xaxis(2,:), xaxis(3,:), 0);
    hold off;
    axis equal;
    grid on;
    
    sensor1_expressedIn_world = zeros(length(x),6);
    sensor2_expressedIn_world = zeros(length(x),6);
    T = eye(4);
    
    % This for-loop determines N-poses of sensor 1 and draws it to the
    % figure. Then it determines sensor 2's pose by calculating sensor1's
    % pose * the transformation between them. Finally, it saves the global
    % (inertial) poses expressed in the world (inertial) frame for each N
    % poses.
    for i = 1:length(x)-1
        T(1:3,1:3) = [xaxis(:,i) yaxis(:,i) zaxis(:,i)]; % * bot_quat_to_matrix(bot_angle_axis_to_quat(pi/3, [1 1 1]));
        T(1:3,4) = [x(i), y(i), z(i)];
        hold on;
        DrawAxis(T, 0.1, 'r', 'b', 'k');
        DrawAxis(T * sensor2_expressedIn_sensor1, 0.1, 'g', 'y', 'c');
        sensor1_expressedIn_world(i,:) = GetState(T);
        sensor2_expressedIn_world(i,:) = GetState(T * sensor2_expressedIn_sensor1);
    end
    hold off;
    
    % This code then calculates the transformation between states (poses) for
    % each sensor. These are basically calculating the transformation from
    % one pose to the next.
    %------------------My Change---------------------------------
    [sensor1_expressedIn_prevSensor1,E_sensor1_expressedIn_prevSensor1] = MakeRelState(sensor1_expressedIn_world);
    [sensor2_expressedIn_prevSensor2, E_sensor2_expressedIn_prevSensor2] = TransformDiffHomo( sensor2_expressedIn_sensor1, sensor1_expressedIn_prevSensor1 );
    %------------------My Change---------------------------------
    
    % This code just changes the final pose in the world coordinates to the
    % origin. It is not used in the final object that is created.
    sensor2Initial_expressedIn_world = GetState(GetHomoTransform(sensor1_expressedIn_world(1,:)) * sensor2_expressedIn_sensor1);
    [~,sensor1check_expressedIn_world] = MakeAbsStates(sensor1_expressedIn_world(1,:), [], sensor1_expressedIn_prevSensor1, []);
    [~,sensor2check_expressedIn_world] = MakeAbsStates(sensor2Initial_expressedIn_world, [], sensor2_expressedIn_prevSensor2, []);
    
%     figure(2);
%     plot3(x,y,z);
%     axis equal;
%     grid on;
%     hold on;
%     for i = 1:size(sensor1check_expressedIn_world,1)
%         T1 = GetHomoTransform(sensor1check_expressedIn_world(i,:));
%         DrawAxis(T1, 0.1, 'r', 'b', 'k');
%         T2 = GetHomoTransform(sensor2check_expressedIn_world(i,:));
%         DrawAxis(T2, 0.1, 'g', 'y', 'c');
%     end
%     hold off;
    
    % generate random covariances
    %RandStream.setDefaultStream(RandStream('mt19937ar','seed',0)); % set the random seed so we always get the same random values
        
    std_sensor1_per_unit = [ones(1,3)*sigma_tra,ones(1,3)*sigma_rot];
    std_sensor2_per_unit = [ones(1,3)*sigma_tra,ones(1,3)*sigma_rot];
    
    % Creating our covariances that we use in the observation runs
    cov1 = zeros(size(sensor1_expressedIn_prevSensor1,1), 36);
    cov2 = zeros(size(sensor1_expressedIn_prevSensor1,1), 36);
    
    
    for i = 1:size(sensor1_expressedIn_prevSensor1,1)
        
%         motion1 = abs(sensor1_expressedIn_prevSensor1(i,:));       
%         motion2 = abs(sensor2_expressedIn_prevSensor2(i,:));
        
        S1 = diag((std_sensor1_per_unit).^2) ;
        S2 = diag((std_sensor2_per_unit).^2) ;
        S1 = triu(S1)+triu(S1,1)';
        S2 = triu(S2)+triu(S2,1)';
        cov1(i,:) = reshape(S1,1,[]);
        cov2(i,:) = reshape(S2,1,[]);
        
        
    end
    
    %------------------My Change---------------------------------
    T_calib = GetHomoTransform(calib);
    E_calib = tran2vec(T_calib);
    %------------------My Change---------------------------------
    
    % Creating an object called "run". It contains two sub-objects called
    % "true" and "observations". In true, it saves our generated poses and
    % transformations that we created above.
%     numberOfRuns = 10;%400;
    run.true.sensor1_expressedIn_prevSensor1 = sensor1_expressedIn_prevSensor1;
    run.true.sensor2_expressedIn_prevSensor2 = sensor2_expressedIn_prevSensor2;
    run.true.sensor1_expressedIn_world = sensor1_expressedIn_world;
    run.true.sensor2_expressedIn_world = sensor2_expressedIn_world;
    run.true.cov1 = cov1;
    run.true.cov2 = cov2;
    run.true.calib = calib;
    
    %------------------My Change---------------------------------
    % Creating the format required for Motion Calibration Project
    run.true.E_sensor1_expressedIn_prevSensor1 = E_sensor1_expressedIn_prevSensor1;
    run.true.E_sensor2_expressedIn_prevSensor2 = E_sensor2_expressedIn_prevSensor2;
    run.true.E_calib = E_calib;
    %------------------My Change---------------------------------
    
    % In observations, it creates 10 (we can set this amount) runs with added
    % noise/covariance. These runs each contain the
    % sensorX_expressedIn_prevSensorX matrix for X =1,2. This will allow us
    % to test the effect of noise on the accuracy of a calibration method.
    for i = 1:numberOfRuns
        run.observations{i}.sensor1_expressedIn_prevSensor1 = SampleVelocitiesWithCovariance(run.true.sensor1_expressedIn_prevSensor1, run.true.cov1);
        run.observations{i}.sensor2_expressedIn_prevSensor2 = SampleVelocitiesWithCovariance(run.true.sensor2_expressedIn_prevSensor2, run.true.cov2);
        
        %------------------My Change---------------------------------
        run.observations{i}.E_sensor1_expressedIn_prevSensor1 = SampleVelocitiesWithCovariance(run.true.E_sensor1_expressedIn_prevSensor1, run.true.cov1);
        run.observations{i}.E_sensor2_expressedIn_prevSensor2 = SampleVelocitiesWithCovariance(run.true.E_sensor2_expressedIn_prevSensor2, run.true.cov2);
        %------------------My Change---------------------------------
%         figure(100+i);
%         DisplayRun(run,0);
%         hold on;
%         DisplayRun(run,i);
%         hold off;
    end
        
%     if ( exist('matFilename', 'var') )
%         save(matFilename, 'run');
%     end

function observed_pos_expressedIn_prevPos = SampleVelocitiesWithCovariance(true_pos_expressedIn_prevPos, true_pos_expressedIn_prevPos_cov)

    observed_pos_expressedIn_prevPos = zeros(size(true_pos_expressedIn_prevPos));
    for i = 1:size(observed_pos_expressedIn_prevPos,1)
        c = reshape(true_pos_expressedIn_prevPos_cov(i,:),6,6);
        if nnz(c) == 0
            observed_pos_expressedIn_prevPos(i,:) = true_pos_expressedIn_prevPos(i,:);
        else 
            observed_pos_expressedIn_prevPos(i,:) = mgd(1, 6, true_pos_expressedIn_prevPos(i,:), c);      
        end
          
    end

    
