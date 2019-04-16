function [ estVecT ] = roughT_new( sensorData, rotVec )
%ROUGHT Uses least squares approach to find translation estimate

estVecT = zeros(size(sensorData,1),3);
% Vel is 1, IMU is 2 
% Vel
t_v = sensorData{1}.T_Skm1_Sk(:, 1:3)';
% r_v = sensorData{1}.T_Skm1_Sk(:, 4:6);
% IMU
t_i = sensorData{2}.T_Skm1_Sk(:, 1:3)';
% r_i = sensorData{2}.T_Skm1_Sk(:, 4:6);

t_v_iv = zeros(3,size(t_i,2));
% Rotation matrix from  (Vel to IMU)
RMat = zeros(3,3,size(sensorData,1));
for j = 1:size(RMat,3)                    % j = 2
    RMat(:,:,j) = vec2rot(rotVec(j,:)');
end
%
%s = size(sensorData{1}.T_Skm1_Sk,1);      % s = 11
weight =zeros(size(sensorData{1}.T_Skm1_Sk,1),3);


for i = 1:size(sensorData{1}.T_Skm1_Sk,1)
    t_v_iv(:,i)=t_v(:,i)-RMat(:,:,2)'*t_i(:,i);
    weight(i,:) = sqrt(sensorData{1}.T_Cov_Skm1_Sk(i,1:3).^2 + ...
                               sensorData{2}.T_Cov_Skm1_Sk(i,1:3).^2);
end

% sum_w = sum(weight,1); 
% w = weight./sum_w;      % small covariance -> big weight 
w=1./weight;
t_v_iv = t_v_iv.*w';

estVecT(2,:) = sum(t_v_iv,2)./sum(w,1)';


end