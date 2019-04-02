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
function [times, sensor_expressedIn_world] = MakeAbsStates(initialState, initialTime, sensor_expressedIn_prevSensor, stateTimes)

    sensor_expressedIn_world = zeros(size(sensor_expressedIn_prevSensor,1)+1, 6);           
    sensor_expressedIn_world(1,:) = initialState;
    curPose = GetHomoTransform(initialState);
    
    for i = 1:size(sensor_expressedIn_prevSensor,1)
        curPose = curPose * GetHomoTransform(sensor_expressedIn_prevSensor(i,:));
        sensor_expressedIn_world(i+1,:) = GetState(curPose);
    end
    
    times = [initialTime; stateTimes];