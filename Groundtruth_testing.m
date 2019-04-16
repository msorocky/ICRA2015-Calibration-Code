tformIdx = 1;
%% process velodyne
load('zeusVelData.mat');          % data obtained from genKittiVel/genKittiVel.m
sensorData{tformIdx,1} = velData; % This would be line 2 in the paper's Algorithm 1 
tformIdx = tformIdx + 1;

%% process nav
load('zeusNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

T_i=sensorData{2}.T_Skm1_Sk(500,1:3)';
T_v=sensorData{1}.T_Skm1_Sk(500,1:3)';

T_i=[T_i;1];
T_v=[T_v;1];

% T_I=vec2tran(T_i');

% T_V=vec2tran(T_v');
% groundtruth Trasformation
c1 = [0.0, 0.0, 1.571];
r1 = [0.000, 0.046, 0.399]';

c2=[-0.000, -0.000, -1.571];
r2 = [0.000, -0.046, -0.399]';

C1 = vec2rot(c1')
C2 = vec2rot(c2')

T1 = [C1,r1;[0,0,0,1]];
T2 = [C1,r2;[0,0,0,1]];
T3 = [C2,r1;[0,0,0,1]];
T4 = [C2,r2;[0,0,0,1]];

  
% Err1= T_v - T1*T_i
% Err2= T_v - T2*T_i
%  Err3= T_v - T3*T_i
%  Err4= T_v - T4*T_i

Err_I2V= T_v - T1*T_i
Err_V2I= T_i - T4*T_v
% Err_V2I= T_i - T3*T_v
