clear all
close all
clc
% Lots of solvers. Wrong calculation of theta
% https://arxiv.org/pdf/1711.07300.pdf
% Approach QP
% https://arxiv.org/pdf/1903.04240.pdf
% https://www.researchgate.net/publication/308737646_From_linear_to_nonlinear_MPC_bridging_the_gap_via_the_real-time_iteration
% from Stanford
% Vehicle formulation with trajectory error
% https://arxiv.org/pdf/1903.08818.pdf
% https://www.ixueshu.com/document/c5b30532ff6d500304102187c086ac26318947a18e7f9386.html
% Roborace
% https://github.com/TUMFTM/global_racetrajectory_optimization/blob/master/opt_mintime_traj/src/opt_mintime.py
% With weight transfer
% Model Predictive Stabilization Control of High-Speed Autonomous Ground Vehicles Considering the Effect of Road Topography
%https://www.mdpi.com/2076-3417/8/5/822/pdf
%% Load track
load('.\Tracks\track.mat')
TrackScale = 1;

track.center = track.center * TrackScale;
track.outer = track.outer * TrackScale;
track.inner = track.inner * TrackScale;

figure(1);
plot(track.outer(1,:),track.outer(2,:),'r')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')
hold on
plot(track.center(1,:),track.center(2,:),'k.')
hold on
%% Parameterized spline

% NTrack = size(track.center,2);
% theta_param = zeros(1,NTrack);
% 
% for i=2:1:NTrack
%     theta_param(i) = sqrt( (track.center(1,i)-track.center(1,i-1))^2 + (track.center(2,i)-track.center(2,i-1))^2 );
% end
% psx = spline(theta_param,track.center(1,:));
% psy = spline(theta_param,track.center(2,:));


% Load library


%,'addheader','SislNurbs.h','includepath','C:\Workspaces\MPC\SISL-master\app'
%C:\Workspaces\MPC\SISL-master\include

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
car = Vehicle_v1();
car.Jacobian_function();
car.CreateTrack(track);
tSim = 20;
tHorizon = 3;


% Initial matrix state
startIdx = 63;
car.x0 = track.center(1,startIdx);
car.y0 = track.center(2,startIdx);
[car.s0, car.e0] =  car.CalculateTrackdistance(car.x0,car.y0);
car.u0 = 20.0;
car.v0 = 0.0;
car.r0 = 0.0;
car.d_phi = 0.0;
trackWidth = 5;
% [theta, ~] = findTheta([track.center(1,startIdx),track.center(2,startIdx)],track.center,traj.ppx.breaks,trackWidth,startIdx);
% car.a_heading0 = atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta));
% param_dist = [0, 0];
% [~,~,~,param_dist] =  calllib('SislNurbs','closestpoint',Track_Nurbs, [car.x0, car.y0],param_dist );
% 
car.a_heading0 = car.CalculateTrackPhi(car.s0);
car.a_wheel_angle0 = 0.0;

% Test Nurbs vs Splines
% for theta=1:3000
%     test1(theta) = atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta));
%     position = [0, 0, 0, 0];
%     [~,~,position]=calllib('SislNurbs','interrogateNURBS',Track_Nurbs,theta,position);
%     xtest(theta)=position(1);
%     ytest(theta)=position(2);
%     test2(theta) = calllib('SislNurbs','CalculateDerivate',Track_Nurbs,theta);
% end
% figure(1)
% hold on
% plot(xtest,ytest,'g.')
% 
% figure(2)
% plot(1:3000,test1,'b.')
% hold on
% plot(1:3000,test2,'r.')


%% Main Loop
NHorizon = tHorizon/car.delta_t;
dist_weights=ones(1,NHorizon);
head_weights=ones(1,NHorizon)*50;

%Init aSteer
input_vSteering = 0.0 .*ones(1,NHorizon);
% cost = Function_Cost(car,tHorizon,input_vSteering,dist_weights,head_weights,Track_Nurbs);
[StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,input_vSteering);
% plot( CarStates(1,:),CarStates(2,:),'b.')
% Checking state calculation against exact NURBs calculation
% for i=1:NHorizon
%     [real_s,real_e] =  car.CalculateTrackdistance(CarStates(1,i),CarStates(2,i));
%     figure(2)
%     plot(i,real_s,'b.')
%     hold on
%     plot(i,CarStates(3,i),'r.')
%     figure(3)
%     plot(i,real_e,'b.')
%     hold on
%     plot(i,-StateVariables(4,i),'r.')
% end
% Checking linearisation
% x_new(:,1)=StateVariables(:,1);
% for i=1:NHorizon
%     [Ak,Bk,gk] = car.DiscretizedLinearizedMatrices(dot_StateVariables(:,i)',StateVariables(:,i)',input_vSteering(i));
%     x_new(:,i+1) = Ak *StateVariables(:,i)+Bk*input_vSteering(i)+gk;
% end
% %Car states: v r d_phi e a_wheel_angle
% figure(2)
% plot(1:NHorizon,StateVariables(1,:),'b.')
% hold on
% plot(1:NHorizon,x_new(1,1:NHorizon),'r.')



% Optimisation - First step
% Car states: v r d_phi e a_wheel_angle
x0 =[car.v0;car.r0;car.d_phi0;car.e0;car.a_wheel_angle0];

vSteering = zeros(1,NHorizon); %aSteering = zeros(1,NHorizon);
% %Initial guess based on heading angle derivative
% a_steereing_angle_guess = zeros(1,NHorizon);
% for iInitialGuess = 2: NHorizon
%     long_dist_guess = car.s0 + car.delta_t*car.u0; %  param dist
%     a_steereing_angle_guess(iInitialGuess) = car.CalculateTrackPhi(long_dist_guess);
%     %a_steereing_angle_guess(iInitialGuess) = atan2(ppval(traj.dppy,param_dist(1)),ppval(traj.dppx,param_dist(1)));%calllib('SislNurbs','CalculateDerivate',Track_Nurbs,param_dist(1));
%     if(iInitialGuess>1)
%         vSteering(iInitialGuess) = (a_steereing_angle_guess(iInitialGuess)-a_steereing_angle_guess(iInitialGuess-1))/car.delta_t;
%     end
% end
% Control state: v_wheel_angle
u0 = vSteering(1);

[ X,U,info ] = Function_Cost_v1(car,tHorizon,vSteering,x0,u0);

[StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,U);
% plot( CarStates(1,:),CarStates(2,:),'m.')
% figure(2);plot(1:NHorizon,StateVariables(5,:));title('steering')
% figure(3);plot(1:NHorizon,StateVariables(4,:));title('lateral error')
% figure(4);plot(1:NHorizon,StateVariables(3,:));title('heading error')

%% Actual Simulation
% Initialisation (Simulated car = Initial control car)
car_sim = Vehicle_v1();
car_sim.CreateTrack(track);

car_sim.x0 = car.x0;
car_sim.y0 = car.y0;
car_sim.u0 = car.u0;
car_sim.a_heading0 = car.a_heading0;
% Car states: v r d_phi e a_wheel_angle
car_sim.v0 = car.v0;
car_sim.r0 = car.r0;
car.d_phi0 = 0.0; % always 0 if a_heading0 = track heading at x0,y0 (s0) 
[car_sim.s0, car_sim.e0] =  car.CalculateTrackdistance(car_sim.x0,car_sim.y0);
car_sim.a_wheel_angle0 = car.a_wheel_angle0;

for iSim = 1:tSim/car.delta_t
    % Original CarState
    if iSim == 1; car_sim.InitVehicle(); end
    % Simulation
    car_sim.Calculate_states(U(1));
    % Update initial car states of controller for optimisation
    % Car states: v r d_phi e a_wheel_angle
    car.v0 = car_sim.v;
    car.r0 = car_sim.r;
    car.d_phi0 = car_sim.d_phi;
    car.e0 = car_sim.e;
    car.a_wheel_angle0 = car_sim.a_wheel_angle;
    
    car.x0 = car_sim.x;
    car.y0 = car_sim.y;
    car.u0 = car_sim.u;
    car.a_heading0 = car_sim.a_heading;
    car.d_phi0 = car_sim.d_phi;
              
    x0 = [car.v0;car.r0;car.d_phi0;car.e0;car.a_wheel_angle0];
    
    % Warm start
    vSteering = U;   
    u0 = vSteering(1);
    % Optimise
    [ X,U,info ] = Function_Cost_v1(car,tHorizon,vSteering,x0,u0);
    x_coord(iSim) = car_sim.x;
    y_coord(iSim) = car_sim.y;
    steering(iSim) = car_sim.a_wheel_angle;
%     figure(1)
%     hold on
%     plot(car_sim.x,car_sim.y,'m.','MarkerSize',15)
end
plot( x_coord,y_coord,'m.')

figure(1)
hold on
plot( CarState(1,:),CarState(2,:),'b.')


%% Unloading NURBS

calllib('SislNurbs','freeNURBS',Track_Nurbs)
unloadlibrary SislNurbs

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
