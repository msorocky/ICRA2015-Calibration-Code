function generate_sim()
%clear all
%close all
%clc
sigma_tra = (10)*eye(3);
sigma_rot = (10)*eye(3);
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
fprintf('Simulated trajectories have been created\n');
save('SimData', 'sim');
end