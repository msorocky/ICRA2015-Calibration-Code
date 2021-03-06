%% The results is: IMU to Vel, treated Vel as the base
clc;
load('repeat_1.mat');

% addAll();
close all;
%convert from position to error
len=length(scansRange);
len1=length(scansRange1);
% groundtruth IMU to Vel
actualT = [0.000, 0.046, 0.399];
actualR = [-0.000, -0.000, -1.571];
% 
% groundtruth Vel to IMU
% actualT = [-0.046, 0.000, -0.399];
% actualR = [0.000, 0.000, 1.571];

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

% deal with the variance 
%
RVar = mean(RVar,1);
RVar = reshape(RVar,3,len)';  % change 
TVar = mean(TVar,1);
TVar = reshape(TVar,3,len1)';  % change 

%convert to angular error in degrees
% for i = 1:len
%     pop = mvnrnd(RErr(i,:),diag(RVar(i,:)),100);
%     temp = zeros(100,3);
%     %use sampling approach to transfer variance
%     for j = 1:100
%         [r,p,y] = dcm2angle(vec2rot(pop(j,1:3)'));
%         temp(j,:) = abs([r,p,y]*180/pi);
%     end
%     RVar(i,1:3) = std(temp);
%     
%     [r,p,y] = dcm2angle(vec2rot(RErr(i,1:3)'));
%     RErr(i,1:3) = abs([r,p,y])*180/pi;
%     [r,p,y] = dcm2angle(vec2rot(RErrEqual(i,1:3)'));
%     RErrEqual(i,1:3) = abs([r,p,y])*180/pi;
% end

TVar = sqrt(TVar);

x = scansRange';   % 1000 ->250
x_T = scansRange1';   % 1000 ->250


addpath('./plotBounds');

%plot
close all
figure
hold on;
title('Error in roll, pitch and yaw','Interpreter','latex','Fontsize',16)
subplot(3,1,1);
boundedline(x,RErr(:,1),RVar(:,1),'o-r');
% plot(x_T,RErr(:,1),'o-r');
hold on
plot(x,RErrEqual(:,1),'x-k');
ylabel('Roll[rad]','Interpreter','latex','Fontsize',15);
axis([10 scansRange(len) 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');
grid on
legend({'variance' 'method' 'weighted least square'},'Interpreter','latex','Fontsize',13)

subplot(3,1,2);
boundedline(x,RErr(:,2),RVar(:,2),'o-g');
% plot(x_T,RErr(:,2),'o-g');
hold on
plot(x,RErrEqual(:,2),'x-k');
ylabel('Pitch[rad]','Interpreter','latex','Fontsize',15);
axis([10 scansRange(len) 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');
grid on
legend({'variance' 'method' 'weighted least square'},'Interpreter','latex','Fontsize',13)

subplot(3,1,3);
boundedline(x,RErr(:,3),RVar(:,3),'o-b');
% plot(x_T,RErr(:,3),'o-b');
hold on
plot(x,RErrEqual(:,3),'x-k');
ylabel('Yaw[rad]','Interpreter','latex','Fontsize',15);
axis([10 scansRange(len) 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');
grid on 
xlabel('Number of Sensor Readings','Interpreter','latex','Fontsize',15)
% ylabel('$x_2$','Interpreter','latex','Fontsize',24)
legend({'variance' 'method' 'weighted least square'},'Interpreter','latex','Fontsize',13)



figure
hold on;
title('Error in X, Y and Z','Interpreter','latex','Fontsize',16)

subplot(3,1,1);
boundedline(x_T,TErr(:,1),TVar(:,1),'o-r');
% plot(x_T,TErr(:,1),'o-r');
hold on
plot(x,TErrEqual(:,1),'x-k');
ylabel('X[m]','Interpreter','latex','Fontsize',15);
axis([10 scansRange1(len) 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');
grid on
legend({'variance' 'method' 'weighted least square'},'Interpreter','latex','Fontsize',13)

subplot(3,1,2);
boundedline(x_T,TErr(:,2),TVar(:,2),'o-g');
% plot(x_T,TErr(:,2),'o-g');
hold on
plot(x,TErrEqual(:,2),'x-k');
ylabel('Y[m]','Interpreter','latex','Fontsize',15);
axis([10 scansRange1(len) 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');
grid on
legend({'variance' 'method' 'weighted least square'},'Interpreter','latex','Fontsize',13)
subplot(3,1,3);
boundedline(x_T,TErr(:,3),TVar(:,3),'o-b');
% plot(x_T,TErr(:,3),'o-b');
hold on
plot(x,TErrEqual(:,3),'x-k');
ylabel('Z[m]','Interpreter','latex','Fontsize',15);
xlabel('Number of Sensor Readings','Interpreter','latex','Fontsize',15)
axis([10 scansRange1(len) 0 1]);
set(gca,'layer','top');
set(gcf,'color','w');
grid on
legend({'variance' 'method' 'weighted least square'},'Interpreter','latex','Fontsize',13)


