
load('Test_1_Res.mat');

R_list = squeeze(RErr);
t_list = squeeze(TErr);

R = mean(R_list,2); % expressed as a vector [roll pitch yaw]
t = mean(t_list,2); % expressed as a vector [x y z]

calib = [t' R'];