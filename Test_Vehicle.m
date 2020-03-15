clear all
close all
clc
% https://arxiv.org/pdf/1711.07300.pdf

%% Load track
load('.\Tracks\track.mat')

figure(1);
plot(track.outer(1,:),track.outer(2,:),'r')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')

%% Fit spline to track
% add spline library
addpath('splines');
addpath('nurbs_toolbox');

% TODO spline function only works with regular spaced points.
% Fix add function which given any center line and bound generates equlally
% space tracks.
[traj, borders] =splinify(track);
tl = traj.ppy.breaks(end);

% store all data in one struct
Track = struct('traj',traj,'borders',borders,'track_center',track.center,'tl',tl);

for i=1:2:666*2
    knots(i)= (i-1)/1331;
    knots(i+1)=(i-1)/1331;
end
knots(1331)=1;
knots(1332)=1;
TrackSpline = nrbmak(track.center,knots); 
% nrbplot(TrackSpline, 2);

%% Input Parameters and class construction
car = Vehicle;
tSim = 5;
tHorizon = 10;

% Initial matrix state
startIdx = 60;
car.x0 = track.center(1,startIdx);
car.y0 = track.center(2,startIdx);
car.u0 = 20.0;
car.v0 = 0.0;
car.r0 = 0.0;
trackWidth = 5;
[theta, ~] = findTheta([track.center(1,startIdx),track.center(2,startIdx)],track.center,traj.ppx.breaks,trackWidth,startIdx);
car.a_heading0 = atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta));


%% Main Loop
NHorizon = tHorizon/car.delta_t;

dist_weights=ones(1,NHorizon);
head_weights=ones(1,NHorizon);

%Init aSteer
% CarState = car.RunSimulation(tHorizon,aSteering);
input_Steering = 0.01.*ones(1,NHorizon);
cost = Function_Cost(car,tHorizon,input_Steering,dist_weights,head_weights,traj,track,trackWidth);
CarState =  car.RunSimulation(tHorizon,input_Steering);
% plot( CarState(1,:),CarState(2,:),'b.')
% figure(2)
% plot(cost)

% Optimisation
lb = -ones(1,NHorizon);
ub = ones(1,NHorizon);
A = [];
b = [];
Aeq = [];
beq = [];
x0 = zeros(1,NHorizon); %aSteering = zeros(1,NHorizon);
options = optimoptions('fmincon','Display','iter');
nonlcon = [];

cost = @(aSteering)Function_Cost(car,tHorizon,aSteering,dist_weights,head_weights,traj,track,trackWidth);


x = fmincon(cost,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

CarState =  car.RunSimulation(tHorizon,x);
plot( CarState(1,:),CarState(2,:),'b.')



% plot( CarState(1,:),CarState(2,:),'b.')

% figure(1)
% plotyy(x,y,x,asteering)
% title('X vs Y and x vs asteering')
% figure(2)
% plot(time,vy,'r.')
% title('time vs vy')
% figure(3)
% plot(time,yaw*180/pi,'r.')
% title('time vs yaw_rate')
% figure(4)
% plot(time,asteering,'r.')
% title('time vs asteering')
