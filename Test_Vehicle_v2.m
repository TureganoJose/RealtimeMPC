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
% Good definition of Lyapunov stability
% https://www.sciencedirect.com/science/article/pii/S2405896316302191
% Cambered road
% Model Predictive Stabilization Control of High-Speed
% Autonomous Ground Vehicles Considering the Effect of Road Topography
% Two proposal: 1) Non-linear MPC (solving non-linear optimization problem at each time step rather than horizon) and 2) LTV MPC
% https://borrelli.me.berkeley.edu/pdfpub/pub-2.pdf
% Explicit MPC
% https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5849315/
% Good second derivatives of e and d_phi. Decent stability analysis
% https://ddl.stanford.edu/sites/g/files/sbiybj9456/f/publications/2012_Thesis_Kritayakirana_Autonomous_Vehicle_Control_at_the_Limits_of_Handling.pdf
%% Load track
load('.\Tracks\track.mat')
TrackScale = 1;

track.center = track.center * TrackScale;
track.outer = track.outer * TrackScale;
track.inner = track.inner * TrackScale;


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
car = Vehicle_v2();
car.CreateTrack(track);
tSim = 100;
tHorizon = 2.5;


% Initial matrix state
startIdx = 420;
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
input_vSteering(1:20)=0.1;
[StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,input_vSteering);
hold on
figure(1)
subplot(2,1,1)
plot(CarStates(1,:),CarStates(2,:),'b.')
title('X vs Y')
subplot(2,1,2)
plot((1:NHorizon)*car.delta_t,StateVariables(5,:),'b.')
title('time vs asteering')
figure(2)
subplot(2,2,1)
plot(CarStates(8,1:end),CarStates(12,1:end),'b')
title('Fy vs slip angle FL')
subplot(2,2,2)
plot(CarStates(9,1:end),CarStates(13,1:end),'b')
title('Fy vs slip angle FR')
subplot(2,2,3)
plot(CarStates(10,1:end),CarStates(14,1:end),'b')
title('Fy vs slip angle RL')
subplot(2,2,4)
plot(CarStates(11,1:end),CarStates(15,1:end),'b')
title('Fy vs slip angle RR')
figure(3)
subplot(2,2,1)
plot((1:NHorizon)*car.delta_t,CarStates(4,:))
title('time vs Fz FL')
subplot(2,2,2)
plot((1:NHorizon)*car.delta_t,CarStates(5,:))
title('time vs Fz FR')
subplot(2,2,3)
plot((1:NHorizon)*car.delta_t,CarStates(6,:))
title('time vs Fz RL')
subplot(2,2,4)
plot((1:NHorizon)*car.delta_t,CarStates(7,:))
title('time vs Fz RR')

figure(4)
plot((1:NHorizon)*car.delta_t,dot_StateVariables(1,:) + 40*StateVariables(2,:))
title('time vs lateral acc')



