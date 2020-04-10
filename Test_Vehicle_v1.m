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
% Good complete example of LTV MPC with 4 wheels and slip angle constrains
% Vehicle Path Tracking LTV-MPC Controller Parameter Selection Considering CPU Computational Load
% https://www.researchgate.net/publication/329438678_Vehicle_Path_Tracking_LTV-MPC_Controller_Parameter_Selection_Considering_CPU_Computational_Load
% Actual weight values are given here:
% https://iopscience.iop.org/article/10.1088/1742-6596/783/1/012028
% Minimum distance
% https://ieeexplore.ieee.org/document/8695742 
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


%% Fit spline to track
% add spline library
addpath('splines');
addpath('nurbs_toolbox');

for i=1:2:666*2
    knots(i)= (i-1)/1331;
    knots(i+1)=(i-1)/1331;
end
knots(1331)=1;
knots(1332)=1;

%% Input Parameters and class construction
car = Vehicle_v1();
car.CreateTrack(track);
tSim = 400;
tHorizon = 2;


% Initial matrix state
startIdx = 54;
car.x0 = track.center(1,startIdx);
car.y0 = track.center(2,startIdx);
[car.s0, car.e0] =  car.CalculateTrackdistance(car.x0,car.y0);
car.u0 = 40.0;
car.u = car.u0;
car.v0 = 0.0;
car.r0 = 0.0;
car.d_phi = 0.0;
trackWidth = 5;


car.a_heading0 = car.CalculateTrackPhi(car.s0);
car.k0 = (car.CalculateTrackPhi(car.s0+0.01)-car.a_heading0)/0.01;
car.a_wheel_angle0 = 0.0;

% Note that the Jacobian is calculated for a specific value of u
% using matlab sym
car.Jacobian_function();


%% Main Loop
NHorizon = tHorizon/car.delta_t;

%Init aSteer
input_vSteering = 0.0 .*ones(1,NHorizon);

% Optimisation - First step
% Car states: v r d_phi e a_wheel_angle

U = zeros(1,NHorizon);
vSteering = U; %aSteering = zeros(1,NHorizon);
x0 = [car.v0;car.r0;car.d_phi0;car.e0;car.a_wheel_angle0];
u0 = vSteering(1);

[ X,U,info ] = Function_Cost_v1(car,tHorizon,vSteering,x0,u0);

[StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,U);

%Checking linearisation
x_new(:,1)=StateVariables(:,1);
x_sol(:,1)=X(:,1);
for i=1:NHorizon
    [Ak,Bk,gk] = car.DiscretizedLinearizedMatrices(dot_StateVariables(:,i),StateVariables(:,i),vSteering(i));
    x_new(:,i+1) = Ak *StateVariables(:,i)+Bk*vSteering(i)+gk;
    x_sol(:,i+1) = Ak *X(:,i)+Bk*U(i)+gk;
end
figure(1)
plot( CarStates(1,:),CarStates(2,:),'m.')



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
car_sim.k0 = (car.CalculateTrackPhi(car.s0+0.01)-car.a_heading0)/0.01;

Optimizer_Inputs = U;
iOpt = 2;
tic
for iSim = 1:tSim/car.delta_t
    % Original CarState
    if iSim == 1; car_sim.InitVehicle(); end
    % Simulation
    car_sim.Calculate_states(Optimizer_Inputs(iOpt));
    % Update initial car states of controller for optimisation
    % Car states: v r d_phi e a_wheel_angle
    car.v0 = car_sim.v;
    car.r0 = car_sim.r;
    car.d_phi0 = car_sim.d_phi;
    car.e0 = car_sim.e;
    car.s0 = car_sim.s;
    car.a_wheel_angle0 = car_sim.a_wheel_angle;
    car.k0 = car_sim.k;
    
    car.x0 = car_sim.x;
    car.y0 = car_sim.y;
    car.u0 = car_sim.u;
    car.a_heading0 = car_sim.a_heading;
    car.d_phi0 = car_sim.d_phi;
              
    x0 = [car.v0;car.r0;car.d_phi0;car.e0;car.a_wheel_angle0];
    
    % Warm start
    vSteering = Optimizer_Inputs;   
    u0 = vSteering(1);
    % Optimise
    [ X,U,info ] = Function_Cost_v1(car,tHorizon,vSteering,x0,u0);
    if info.exitflag == 1
        iOpt = iOpt + 1;
        Optimizer_Inputs = Optimizer_Inputs;
    else
        iOpt = 2;
        Optimizer_Inputs = U;
    end
    x_coord(iSim) = car_sim.x;
    y_coord(iSim) = car_sim.y;
    steering(iSim) = car_sim.a_wheel_angle;

%     figure(1)
%     plot( x_coord(iSim),y_coord(iSim),'m.')
%     hold on
%     
%     figure(3)
%     
%     [long_dist, lat_dist] =  car_sim.CalculateTrackdistance(car_sim.x,car_sim.y);
%     
%     subplot(3,2,1)
%     plot(iSim,car_sim.v,'m.')
%     hold on; title('v vs time')
%     subplot(3,2,2)
%     plot(iSim,car_sim.r,'m.')
%     hold on; title('r vs time')
%     subplot(3,2,3)
%     plot(iSim,car_sim.d_phi,'m.')
%     hold on; title('d_phi vs time')
%     subplot(3,2,4)
%     plot(iSim,abs(car_sim.e),'m.')
%     hold on; title('lateral error vs time')
%     plot(iSim,lat_dist,'r.')
%     hold on;
%     subplot(3,2,5)
%     plot(iSim,car_sim.a_wheel_angle,'m.')
%     hold on; title('Wheel_angle vs time')
%     subplot(3,2,6)
%     plot(iSim,car_sim.s,'m.')
%     hold on
%     plot(iSim,long_dist,'r.')
%     hold on; title('s param');
%     
%     figure(4)
%     subplot(2,2,1)
%     plot(iSim, car_sim.k,'m.')
%     hold on; title('Curvature')
%     subplot(2,2,2)
%     plot(iSim,car_sim.CalculateTrackPhi(long_dist),'m.')
%     hold on; title('track ref heading')
%     subplot(2,2,3)
%     plot(iSim,car_sim.a_heading,'m.')
%     hold on; title('car Heading angle')
%     subplot(2,2,4)
%     plot(iSim,info.QPtime,'m.')
%     hold on; title('QP solving time (s)')
end
toc

%% Unloading NURBS

calllib('SislNurbs','freeNURBS',Track_Nurbs)
unloadlibrary SislNurbs


