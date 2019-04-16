function [ prob ] = systemProbT_new( sensorData, tformMat, estVec, R )
%ROTSYS2VEC Summary of this funcit(i,on goes here
%   Detailed explanait(i,on goes here

      estVec = [0,0,0;estVec];
%       s = size(tformMat,1);   s=2
      prob = 0;
% 1 -> Vel   2 -> IMU
      a=2;  % a= IMU 
      b=1;  % b= Vel

      Ta = [R(:,:,a),estVec(a,:)';[0,0,0,1]];
      Tb = [R(:,:,b),estVec(b,:)';[0,0,0,1]];
      Tab = Tb/Ta;
            
      Rab = Tab(1:3,1:3);    %%%%
      temp = Tab(1:3,4);
            
      VA = sensorData{a}.T_Cov_Skm1_Sk(:,1:3)';
      VB = sensorData{b}.T_Cov_Skm1_Sk(:,1:3)';
            
      estA = sensorData{a}.T_Skm1_Sk(:,1:3)';
      estB = sensorData{b}.T_Skm1_Sk(:,1:3)';
            
      estA = -Rab*estA;
            
      estAB = tformMat{b}(:,1:9);
      estAB(:,1) = estAB(:,1) - 1;
      estAB(:,5) = estAB(:,5) - 1;
      estAB(:,9) = estAB(:,9) - 1;
            
      estAB = [estAB(:,1)*temp(1) + estAB(:,4)*temp(2) + estAB(:,7)*temp(3),...
               estAB(:,2)*temp(1) + estAB(:,5)*temp(2) + estAB(:,8)*temp(3), ...
               estAB(:,3)*temp(1) + estAB(:,6)*temp(2) + estAB(:,9)*temp(3)];           
               
      err = estAB' + estA + estB;
       
      temp = cprobR(err, VA, VB, Rab);
      prob = prob + temp;
end


