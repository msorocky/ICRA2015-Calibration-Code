clear
clc
load('SimulatedData/SimData.mat'); % change this to where your simulations are saved 
HaveNoise=true;
N=1;
if HaveNoise
    Length = size(sim.noisy_observations{1,N}.sensor1_expressedIn_prevSensor1,1);
else
    Length = size(sim.true.sensor1_expressedIn_prevSensor1,1);
end
velData.T_Skm1_Sk=zeros(Length,6);
navData.T_Skm1_Sk=zeros(Length,6);
velData.T_Cov_Skm1_Sk = zeros(Length,6);
navData.T_Cov_Skm1_Sk = zeros(Length,6);

if HaveNoise
    fprintf('Using the noisy simulation data\n');
    velData.T_Skm1_Sk=sim.noisy_observations{1,N}.sensor1_expressedIn_prevSensor1;
    navData.T_Skm1_Sk=sim.noisy_observations{1,N}.sensor2_expressedIn_prevSensor2;
else
    fprintf('Using the true simulation data\n');
    velData.T_Skm1_Sk=sim.true.sensor1_expressedIn_prevSensor1;
    navData.T_Skm1_Sk=sim.true.sensor2_expressedIn_prevSensor2;
end
Groundtruth =sim.true.calib_ground;
for i =1: Length
    % Get the Cov of Veld
    Cov = reshape(sim.true.cov_sensor1(i,:),6,6);
    velData.T_Cov_Skm1_Sk(i,:)=diag(Cov);
    velData.T_Cov_Skm1_Sk(i,1:3)=velData.T_Cov_Skm1_Sk(i,1:3);
    % Get the Cov of Nav
    Cov2 = reshape(sim.true.cov_sensor2(i,:),6,6);
    navData.T_Cov_Skm1_Sk(i,:)=diag(Cov2);
    navData.T_Cov_Skm1_Sk(i,1:3)=navData.T_Cov_Skm1_Sk(i,1:3);
end

velData.type = 'lidar';
navData.type = 'nav';

save('SimulatedData/Sim_Data_noise1.mat', 'velData', 'navData','Groundtruth');
fprintf("Finished")
