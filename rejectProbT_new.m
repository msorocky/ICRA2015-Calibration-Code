function [ keep ] = rejectProbT_new( sensorData, tformMat, estVec, R )
%ROTSYS2VEC Summary of this function goes here
%   k = rejectProbT( {sensorData{1};sensorData{i}}, ...
%      {tformMat{1};tformMat{i}}, tranVec(i,:), RMat(:,:,[1,i]) );

%sensorData={sensorData{nav};sensorData{Vel}}
%tformMat={tformMat{nav};tformMat{Vel}},
%estVec = tranVec(2,:)  this is the tranVec from Vel to IMU
%R = RMat(:,:,[1,i])    this is the rotation matrix from Vel to IMU 
estVec = [0,0,0;estVec];

% We need IMU to Vel

% s = size(tformMat,1);      s=2

keep = true(size(tformMat{1},1),1);
% 1 -> Vel   2 -> IMU
a=2;  % a= IMU 
b=1;  % b= Vel

      Ta = [R(:,:,a),estVec(a,:)';[0,0,0,1]];    % T of imu 
      Tb = [R(:,:,b),estVec(b,:)';[0,0,0,1]];    % T of vel
      Tab = Ta*inv(Tb);   % T of vel to imu 
            
      Rab = Tab(1:3,1:3); % R of vel to imu
      temp = Tab(1:3,4);  % T_vec of vel to imu
            
      VA = sensorData{a}.T_Cov_Skm1_Sk(:,1:3)';  % variance T_vec of imu (k-1) to k
      VB = sensorData{b}.T_Cov_Skm1_Sk(:,1:3)';  % variance T_vec of Vel (k-1) to k
            
      estA = sensorData{a}.T_Skm1_Sk(:,1:3)';    % T_vec of imu (k-1) to k
      estB = sensorData{b}.T_Skm1_Sk(:,1:3)';    % T_vec of Vel (k-1) to k
            
      estA = -Rab*estA;
            
      estAB = tformMat{b}(:,1:9);                % estAB = 
      estAB(:,1) = estAB(:,1) - 1;
      estAB(:,5) = estAB(:,5) - 1;
      estAB(:,9) = estAB(:,9) - 1;
            
      temp = (estVec(b,:)' - Rab*estVec(a,:)');
            
      estAB = [estAB(:,1)*temp(1) + estAB(:,4)*temp(2) + estAB(:,7)*temp(3), estAB(:,2)*temp(1) + estAB(:,5)*temp(2) + estAB(:,8)*temp(3), estAB(:,3)*temp(1) + estAB(:,6)*temp(2) + estAB(:,9)*temp(3)];
            
      err = estAB' + estA + estB;     
      err = err';
            
      for k = 1:size(VA,2)
           V = diag(VB(:,k)) + Rab*diag(VA(:,k))*Rab';
           err(k,:) = abs(err(k,:)) ./ sqrt(diag(V))'; 
      end
            
      keep(any(err > 1,2)) = false;
%            keep(any(err > 0.01,2)) = false;
end 


