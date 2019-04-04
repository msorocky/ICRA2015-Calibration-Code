function [ tformMat, tformCov] = readKittiNavWithError( path )
%READVELDATA Reads binary velodyne data

fid = fopen(path, 'r');

in = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','CollectOutput', 1);
in = in{1};
fclose(fid);

% compute scale from first lat value
t = [in(1) in(2) in(3)];

% Quaternion 
q = [in(7) in(4) in(5) in(6)];

R = quat2rotm(q);
tformMat = eye(4);
tformMat(1:3,1:3)  = R;
tformMat(1:3,4) = t;


cov_vec = zeros(36,1);

for k = 1 : 36    
    cov_vec(k) = in(7+k);
end

Cov = reshape(cov_vec, [6, 6])';

tformCov = diag(Cov);

end