clear all
close all
tra_squared = [sqrt(0.007999), sqrt(0.003302), sqrt(0.021322)];
rot_squared = [sqrt(0.001216), sqrt(0.001131), sqrt(0.026699)];
%sigma_tra = diag(tra_squared);
%sigma_rot = diag(rot_squared);
sigma_tra = 0.001*eye(3,3);
sigma_rot = 0.001*eye(3,3);
N = 1000;
numberOfRuns = 10;
% calib
Translation = [-0.046, 0.000, -0.399];
Rotation_RPW = [-0.000, -0.000, -1.571];
calib_Vel2IMU = [Translation Rotation_RPW];
result = Create3DSimulatedData(sigma_tra, sigma_rot, N, numberOfRuns, calib_Vel2IMU);
sim.true.sensor1_expressedIn_prevSensor1 = result.true.E_sensor1_expressedIn_prevSensor1;
sim.true.sensor2_expressedIn_prevSensor2 = result.true.E_sensor2_expressedIn_prevSensor2;
sim.true.calib_ground = result.true.E_calib;
sim.true.cov_sensor1 = result.true.cov1;
sim.true.cov_sensor2 = result.true.cov2;
for i = 1:numberOfRuns
    sim.noisy_observations{i}.sensor1_expressedIn_prevSensor1 = result.observations{i}.E_sensor1_expressedIn_prevSensor1;
    sim.noisy_observations{i}.sensor2_expressedIn_prevSensor2 = result.observations{i}.E_sensor2_expressedIn_prevSensor2;
end

save('SimulatedData/SimData', 'sim');

% save('Data', 'sensor1_expressedIn_prevSensor1', 'sensor2_expressedIn_prevSensor2',...
%     'calib_ground', 'sigma_rot', 'sigma_tra', 'cov_sensor1', 'cov2_sensor2');
% 
% from velodyne to imu:
% Translation: [-0.046, 0.000, -0.399] [m]
% Rotation: in Quaternion [-0.000, -0.000, -0.707, 0.707]
%            in RPY (radian) [-0.000, -0.000, -1.571]
%            in RPY (degree) [-0.000, -0.000, -90.000]
% 
% From imu to velodyne:
% - Translation: [0.000, 0.046, 0.399]
% - Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
%            in RPY (radian) [0.000, -0.000, 1.571]
%            in RPY (degree) [0.000, -0.000, 90.000]