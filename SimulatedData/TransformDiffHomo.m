%
% Copyright © 2012, The Massachusetts Institute of Technology. All rights reserved. 
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
function [sensor2_expressedIn_prevSensor2, E_sensor2_expressedIn_prevSensor2] = TransformDiffHomo(sensor2_expressedIn_sensor1, sensor1_expressedIn_prevSensor1)

    % this function does the equivalent of taking a set of differential states, coverting them to absolute states,
    % applying  rigid body transform to each state, and then coverting back to differential states
    
    sensor2_expressedIn_prevSensor2 = zeros(size(sensor1_expressedIn_prevSensor1));
    
    %------------------My Change---------------------------------
    E_sensor2_expressedIn_prevSensor2 = zeros(size(sensor1_expressedIn_prevSensor1));
    %------------------My Change---------------------------------
    
    sensor1_expressedIn_sensor2 = inv(sensor2_expressedIn_sensor1);
    for i = 1:size(sensor1_expressedIn_prevSensor1,1)
        %------------------My Change---------------------------------
        [sensor2_expressedIn_prevSensor2(i,:), E_sensor2_expressedIn_prevSensor2(i,:)] = GetState( sensor1_expressedIn_sensor2 * GetHomoTransform(sensor1_expressedIn_prevSensor1(i,:)) * sensor2_expressedIn_sensor1 );
        %------------------My Change---------------------------------
    end