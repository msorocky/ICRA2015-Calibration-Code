%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The results are IMU to Velydne
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clc;
load('SimulatedData/Sim_Results_noise1.mat');

% Convert from position to error
% The groundtruth data should change each time

% Groundtruth for Kitti 9_30_drive_20

% Groundtruth for simulation

actualR = [0.000, -0.000, 1.571];  
% actualR = Groundtruth(4:6);

actualT = [0.000, 0.046, 0.399];
% actualT = Groundtruth(1:3);
% Calculating the errors of each iteration within each algorithm run
RErr = RErr - repmat(actualR,[size(RErr,1),1,size(RErr,3)]);
RErrEqual = RErrEqual - repmat(actualR,[size(RErrEqual,1),1,size(RErrEqual,3)]);
TErr = TErr - repmat(actualT,[size(TErr,1),1,size(TErr,3)]);
TErrEqual = TErrEqual - repmat(actualT,[size(TErrEqual,1),1,size(TErrEqual,3)]);

%get means
RErr = mean(abs(RErr),1);
RErr = reshape(RErr,3,size(RErr,3))';
TErr = mean(abs(TErr),1);
TErr = reshape(TErr,3,size(TErr,3))';

RErrEqual = mean(abs(RErrEqual),1);
RErrEqual = reshape(RErrEqual,3,size(RErrEqual,3))';
TErrEqual = mean(abs(TErrEqual),1);
TErrEqual = reshape(TErrEqual,3,size(TErrEqual,3))';

% variance computation 
while 0 

RVar = mean(RVar,1);
RVar = reshape(RVar,3,20)';  % 
TVar = mean(TVar,1);
TVar = reshape(TVar,3,20)';  % 

%convert to angular error in degrees
for i = 1:20
    pop = mvnrnd(RErr(i,:),diag(RVar(i,:)),100);
    temp = zeros(100,3);
    %use sampling approach to transfer variance
    for j = 1:100
        [r,p,y] = dcm2angle(vec2rot(pop(j,1:3)'));
        temp(j,:) = abs([r,p,y]*180/pi);
    end
    RVar(i,1:3) = std(temp);
    
    [r,p,y] = dcm2angle(vec2rot(RErr(i,1:3)'));
    RErr(i,1:3) = abs([r,p,y])*180/pi;
    [r,p,y] = dcm2angle(vec2rot(RErrEqual(i,1:3)'));
    RErrEqual(i,1:3) = abs([r,p,y])*180/pi;
end

TVar = sqrt(TVar);
end


% Error computation
for i = 1:20
    [r,p,y] = dcm2angle(vec2rot(RErr(i,1:3)'));
    % For simulation data, already degree
    % RErr(i,1:3) = abs([r,p,y])*180/pi; 
    [r,p,y] = dcm2angle(vec2rot(RErrEqual(i,1:3)'));
    % RErrEqual(i,1:3) = abs([r,p,y])*180/pi;
end

x = (50:50:1000)';   % 

% addpath('./plotBounds');

%plot
close all
figure
hold on;

% Do not consider variance first

subplot(3,1,1);
%boundedline(x,RErr(:,1),RVar(:,1),'o-r');
plot(x,RErr(:,1),'o-r');
hold on 
plot(x,RErrEqual(:,1),'x-k');
ylabel('Roll');
axis([10 1000 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');

subplot(3,1,2);
%boundedline(x,RErr(:,2),RVar(:,2),'o-g');
plot(x,RErr(:,2),'o-g');
hold on
plot(x,RErrEqual(:,2),'x-k');
ylabel('Pitch');
axis([10 1000 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');

subplot(3,1,3);
%boundedline(x,RErr(:,3),RVar(:,3),'o-b');
plot(x,RErr(:,3),'o-b');
hold on
plot(x,RErrEqual(:,3),'x-k');
ylabel('Yaw');
xlabel('Number of Sensor Readings');
axis([10 1000 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');

figure
hold on;

subplot(3,1,1);
%boundedline(x,TErr(:,1),TVar(:,1),'o-r');
plot(x,TErr(:,1),'o-r');
hold on
plot(x,TErrEqual(:,1),'x-k');
ylabel('X');
axis([10 1000 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');

subplot(3,1,2);
%boundedline(x,TErr(:,2),TVar(:,2),'o-g');
plot(x,TErr(:,2),'o-g');
hold on
plot(x,TErrEqual(:,2),'x-k');
ylabel('Y');
axis([10 1000 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');

subplot(3,1,3);
%boundedline(x,TErr(:,3),TVar(:,3),'o-b');
plot(x,TErr(:,3),'o-b');
hold on 
plot(x,TErrEqual(:,3),'x-k');
ylabel('Z');
xlabel('Number of Sensor Readings');
axis([10 1000 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');



