function [ prob ] = systemProbT_new( sensorData, estVec, R )
%ROTSYS2VEC Summary of this funcit(i,on goes here
%   Detailed explanait(i,on goes here
%     systemProbT_new( sensorData, tformMat, estVec, RMat_new)
%      estVec = [0,0,0;estVec];
%       s = size(tformMat,1);   s=2
      prob = 0;
% 1 -> Vel   2 -> IMU
%       a=2;  % a= IMU 
%       b=1;  % b= Vel

      % Vel
      t_v = sensorData{1}.T_Skm1_Sk(:, 1:3)';
      % IMU
      t_i = sensorData{2}.T_Skm1_Sk(:, 1:3)';
      % Err
      Err=zeros(4,size(sensorData{1}.T_Skm1_Sk,1));
      % T matrix from  (Vel to IMU)
      
      T_IMU2Vel = [R estVec';[0 0 0 1]];

      for i = 1:size(sensorData{1}.T_Skm1_Sk,1)
          Err(:,i)=[t_v(:,i);1] - T_IMU2Vel*[t_i(:,i);1];
      end
      VA = sensorData{1}.T_Cov_Skm1_Sk(:,1:3)';
      VB = sensorData{2}.T_Cov_Skm1_Sk(:,1:3)';
      
%       err = estAB' + estA + estB;
      err=Err(1:3,:); 
      temp = cprobR(err, VA, VB, R);
      prob = prob + temp;
end


