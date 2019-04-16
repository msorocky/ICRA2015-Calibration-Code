function [ sensorData ] = findInT( sensorData, tranVec, rotVec )
%FINDINT Filters for inliers based on translation

%convert rot vector to rotmats
RMat = zeros(3,3,size(sensorData,1));
for i = 1:size(RMat,3)
    RMat(:,:,i) = vec2rot(rotVec(i,:)');
end

R_Vel2IMU=RMat(:,:,2);
R=R_Vel2IMU';  % R is IMU to Vel

%get matrix form of transformations
tformMat = cell(size(sensorData));
for i = 1:size(sensorData,1)
    tformMat{i} = zeros(size(sensorData{i}.T_Skm1_Sk,1),12);
    for j = 1:size(sensorData{i}.T_Skm1_Sk,1)
        temp = vec2tran(sensorData{i}.T_Skm1_Sk(j,:)');
        r = temp(1:3,1:3);
        t = temp(1:3,4);
        tformMat{i}(j,:) = [r(:)' t(:)'];
    end
end

%find points to keep
keep = true(size(sensorData{1}.T_Skm1_Sk,1),1);
for i = 2:size(sensorData,1)      % i=2
    %k = 0;
    [k_xy,k_z] = rejectProbT_new( {sensorData{1};sensorData{i}}, tranVec(i,:), R );
    keep = and(keep,k_xy);

end

     for i=2:size(sensorData{1}.T_Skm1_Sk,1)
        if k_z(i) == false
           sensorData{1}.T_Skm1_Sk(i,3)=sensorData{1}.T_Skm1_Sk(i-1,3);
           sensorData{2}.T_Skm1_Sk(i,3)=sensorData{2}.T_Skm1_Sk(i-1,3);
        end
     end

%remove points with large errors
for i = 1:size(sensorData,1)
    sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(keep,:);
    sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(keep,:);
end

