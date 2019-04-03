%
% Copyright � 2012, The Massachusetts Institute of Technology. All rights reserved. 
%
% THE LICENSOR EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS 
% SOFIWARE AND DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR ANY PARTICULAR PURPOSE, NON- INFRINGEMENT AND WARRANTIES OF 
% PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM COURSE OF 
% DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH 
% RESPECT TO THE USE OF THE SOFIWARE OR DOCUMENTATION. Under no circumstances 
% shall the Licensor be liable for incidental, special, indirect, direct or 
% consequential damages, or loss of profits, interruption of business, or 
% related expenses which may arise from use of Software or Documentation, 
% including but not limited to those resulting from defects in Software 
% and/or Documentation, or loss or inaccuracy of data of any kind. 
%
% This software is licensed under the "LIMITED RESEARCH LICENSE (SOURCE
% CODE)" as described in the included LICENSE.txt
%
% Please cite the paper below if you are using this software in your work:
% Brookshire, J.; Teller, S. Extrinsic Calibration from Per-Sensor Egomotion. 
%   Robotics: Science and Systems, 2012.
%

% CREATES THE POSE/TRANSFORMATION MATRIX FROM THE x, y, z, roll, pitch, yaw
function T = GetHomoTransform(xyzrpy)

    if length(xyzrpy)==6
        %row pose is [pos, rpy]
        T = eye(4); % Create a 4x4 identity matrix
        
        % Creates the rotation component of the transformation matrix. This
        % is accomplished by converting the roll, pitch, yaw angles to a
        % quaternion, then computing the rotation matrix from the
        % quaternion.
        T(1:3,1:3) = bot_quat_to_matrix(bot_roll_pitch_yaw_to_quat(xyzrpy(4:6)));
        
        % Creates the translation component of the transformation matrix
        T(1:3,4) = xyzrpy(1:3)';  
    else
        error('wrong size for input, should be [x,y,z,r,p,y]')
    end