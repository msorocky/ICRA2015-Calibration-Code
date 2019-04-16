function [ keep_xy, keep_z] = rejectProbT_new( sensorData, estVec, R )
%ROTSYS2VEC Summary of this function goes here
%   k = rejectProbT( {sensorData{1};sensorData{i}}, ...
%      {tformMat{1};tformMat{i}}, tranVec(i,:), RMat(:,:,[1,i]) );

%sensorData={sensorData{nav};sensorData{Vel}}
%tformMat={tformMat{nav};tformMat{Vel}},
%estVec = tranVec(2,:)  this is the tranVec from Vel to IMU
%R = RMat(:,:,[1,i])    this is the rotation matrix from Vel to IMU 
% estVec = [0,0,0;estVec];
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
%       VA = sensorData{1}.T_Cov_Skm1_Sk(:,1:3)';
%       VB = sensorData{2}.T_Cov_Skm1_Sk(:,1:3)';
%       
%       err = estAB' + estA + estB;
      err=Err(1:3,:); 

% We need IMU to Vel

% s = size(tformMat,1);      s=2

      keep_xy = true(size(sensorData{1}.T_Skm1_Sk,1),1);
      keep_z = true(size(sensorData{1}.T_Skm1_Sk,1),1);
% 1 -> Vel   2 -> IMU

            
      VA = sensorData{1}.T_Cov_Skm1_Sk(:,1:3)';  % variance T_vec of imu (k-1) to k
      VB = sensorData{2}.T_Cov_Skm1_Sk(:,1:3)';  % variance T_vec of Vel (k-1) to k
               
      err = err';
            
      for k = 1:size(VA,2)
           V = diag(VB(:,k)) + R*diag(VA(:,k))*R';
           err(k,:) = abs(err(k,:)) ./ sqrt(diag(V))'; 
      end
         
      err_xy = err(:,1:2);
      err_z = err(:,3);
      
      keep_xy(any(err_xy > 1, 2)) = false;
      keep_z(any(err_z > 0.1 ,2)) = false;
      
%            keep(any(err > 0.01,2)) = false;
end 


