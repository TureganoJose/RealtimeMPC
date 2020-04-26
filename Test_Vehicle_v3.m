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
% Basic vehicle dynamics
% http://publications.lib.chalmers.se/records/fulltext/225751/local_225751.pdf
% More stabililty analysis from a vehicle dynamics point of view
% https://ddl.stanford.edu/sites/g/files/sbiybj9456/f/publications/2011_Thesis_Beal_Applications_of_Model_Predictive_Control_to_Vehicle_Dynamics_for_Active_Safety_and_Stability.pdf
% Comprehensive Phd thesis with 14-dof vehicle model. Some interesting
% thoughts about MPC in AV
% https://www.researchgate.net/publication/335970485_Optimal_Coordination_of_Chassis_Systems_for_Vehicle_Motion_Control
% Shows influence of roll (although it doesn't show the influence of Fz)
% A Switched MPC Lateral Steering Controller Which Considered Tracking Quality and Handling Quality for Autonomous Vehicle
% Good explanation of terminal cost and constraints to ensure
% stability/feasibility (sadly it only works with continious systems rather
% than discrete ones)
% https://d-nb.info/1054135541/34
% General overview of optimal control problems
% https://www.stardust2013.eu/Portals/63/Images/Training/OTS%20Repository/OTS-Talk-Topputo.pdf
% Good explaining the QP. Comparing the QP to Newton-Raphson method
% https://ecal.berkeley.edu/files/ce191/CH02-QuadraticProgramming.pdf
% General overview of MPC, indirect/direct methods for transcription, 
% A_Nonlinear_Model_Predictive_Control_based_Evasive_Manoeuvre_Assist_Function.pdf
% Berckley's model. Mayne 2002 A treatment of sufficient
% stability conditions goes beyond the scope of this work and can be found in the
% surveys (Mayne et al., 2000; Mayne, 2001). 
% https://borrelli.me.berkeley.edu/pdfpub/pub-6.pdf
% This book contains everything
% https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-2nd-printing.pdf
% Model fidelity in MPCs
% http://www-personal.umich.edu/~tersal/papers/paper30.pdf
% 14 DOF vehicle
% https://documents.pub/document/understanding-the-limitations-of-different-vehicle-models-for-roll-dynamics.html
% New solver
% https://alphaville.github.io/optimization-engine/
% good material from italian professor
% http://cse.lab.imtlucca.it/~bemporad/mpc_course.html
% From linear to non-linear MPC
% https://www.researchgate.net/publication/308737646_From_linear_to_nonlinear_MPC_bridging_the_gap_via_the_real-time_iteration
%% Load track
load('.\Tracks\track.mat') %dlc2
TrackScale = 1;

track.center = track.center * TrackScale;
track.outer = track.outer * TrackScale;
track.inner = track.inner * TrackScale;


%% Fit spline to track
% add spline library
addpath('splines');
addpath('nurbs_toolbox');
NTrack = size(track.center  ,2);
for i=1:2:NTrack*2
    knots(i)= (i-1)/(NTrack*2-1);
    knots(i+1)=(i-1)/(NTrack*2-1);
end
knots(NTrack*2-1)=1;
knots(NTrack*2)=1;

%% Input Parameters and class construction
car = Vehicle_v3();
car.CreateTrack(track);
tSim = 50;
tHorizon = 2;


% Initial matrix state
startIdx = 35;
car.x0 = track.center(1,startIdx);
car.y0 = track.center(2,startIdx);
[car.s0, car.e0] =  car.CalculateTrackdistance(car.x0,car.y0);
car.u0 = 33.333;
car.u = car.u0;
car.v0 = 0.0;
car.r0 = 0.0;
car.d_phi = 0.0;
NHorizon = tHorizon/car.delta_t;

car.a_heading0 = car.CalculateTrackPhi(car.s0);
car.k0 = (car.CalculateTrackPhi(car.s0+0.01)-car.a_heading0)/0.01;
car.a_wheel_angle0 = 0.0;


%% Main Loop

%Init aSteer
input_vSteering = 0.0 .*ones(1,NHorizon);

% Optimisation - First step
% Car states: v r d_phi e a_wheel_angle

U = zeros(1,NHorizon);
vSteering = U; %aSteering = zeros(1,NHorizon);
x0 = [car.v0;car.r0;car.d_phi0;car.e0;car.a_wheel_angle0];
u0 = vSteering(1);

[ X,U,info ] = Function_Cost_v2(car,tHorizon,vSteering,x0,u0);

[StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,U);

% %Checking linearisation
% x_new(:,1)=StateVariables(:,1);
% x_sol(:,1)=X(:,1);
% for i=1:NHorizon
%     [Ak,Bk,gk] = car.DiscretizedLinearizedMatrices(dot_StateVariables(:,i),StateVariables(:,i),vSteering(i));
%     x_new(:,i+1) = Ak *StateVariables(:,i)+Bk*vSteering(i)+gk;
%     x_sol(:,i+1) = Ak *X(:,i)+Bk*U(i)+gk;
% end


%% Actual Simulation
% Initialisation (Simulated car = Initial control car)
car_sim = Vehicle_v3();
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

%Initialise logging struct
NSim = tSim/car.delta_t;
logging.track = track;
logging.time = (1:NSim) * car_sim.delta_t;

logging.x_coord = zeros(NSim,1);
logging.y_coord = zeros(NSim,1);
logging.u_opt = zeros(NSim,NHorizon);
    
logging.v = zeros(NSim,1);
logging.r = zeros(NSim,1);
logging.d_phi = zeros(NSim,1);
logging.e = zeros(NSim,1);
logging.s = zeros(NSim,1);
logging.a_wheel_angle = zeros(NSim,1);
logging.v_wheel_angle = zeros(NSim,1);

logging.Fz_fl = zeros(NSim,1);
logging.Fz_fr = zeros(NSim,1);
logging.Fz_rl = zeros(NSim,1);
logging.Fz_rr = zeros(NSim,1);
logging.alpha_fl = zeros(NSim,1);
logging.alpha_fr = zeros(NSim,1);
logging.alpha_rl = zeros(NSim,1);
logging.alpha_rr = zeros(NSim,1);
logging.force_fl = zeros(NSim,1);
logging.force_fr = zeros(NSim,1);
logging.force_rl = zeros(NSim,1);
logging.force_rr = zeros(NSim,1);

logging.a_heading = zeros(NSim,1);
logging.k = zeros(NSim,1);
logging.QPtime = zeros(NSim,1);
logging.Failed = zeros(NSim,1);

%Initialise inputs to simulation
Optimizer_Inputs = U;
iOpt = 2;
tic
for iSim = 1:NSim
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
    [ X,U,info,pre_opt_state, pre_opt_dot_state, pre_opt_car_state,HorizonIter ] = Function_Cost_v3(car,tHorizon,vSteering,x0,u0);
    
%     % Checking discretization/linearization
%     [StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,U);
%     
%     x_opt_linear = zeros(5,NHorizon);
%     x_opt_linear(:,1) = X(:,1);
%     cost_aheading = HorizonIter(1).Qk(3,3) * X(3,1)^2;
%     cost_lat_error = HorizonIter(1).Qk(4,4) * X(4,1)^2;
%     cost_v_steering =  HorizonIter(1).Rk*U(1)^2;
%     for i=2:NHorizon
%         x_opt_linear(:,i)= HorizonIter(i).Ak * X(:,i-1) + HorizonIter(i).Bk*U(i-1)+ HorizonIter(i).gk;
%         cost_aheading = cost_aheading + HorizonIter(i).Qk(3,3) * X(3,i)^2;
%         cost_lat_error= cost_lat_error + HorizonIter(i).Qk(4,4) * X(4,i)^2;
%         cost_v_steering = cost_v_steering + HorizonIter(i).Rk*U(i)^2;
%     end
%     
%     fprintf('error heading %f error lateral %f error steering speed %f \n',cost_aheading, cost_lat_error, cost_v_steering)
%     % Error reference optimized inputs with dynamic simulation
%     % Error 1: Optimum states accordding to QP solver
%     % Error 2: States according to linearisation/discretization
%     error_dis_lin = StateVariables - X;
%     error_dis_lin_opt = StateVariables - x_opt_linear;
%     figure(1)
%     subplot(3,2,1)
%     plot(1:NHorizon,error_dis_lin(1,:),'b.',1:NHorizon,error_dis_lin_opt(1,:),'r.')
%     title('lateral v');grid on
%     subplot(3,2,2)
%     plot(1:NHorizon,error_dis_lin(2,:),'b.',1:NHorizon,error_dis_lin_opt(2,:),'r.')
%     title('yaw rate');grid on
%     subplot(3,2,3)
%     plot(1:NHorizon,error_dis_lin(3,:),'b.',1:NHorizon,error_dis_lin_opt(3,:),'r.')
%     title('dphi');grid on
%     subplot(3,2,4)
%     plot(1:NHorizon,error_dis_lin(4,:),'b.',1:NHorizon,error_dis_lin_opt(4,:),'r.')
%     title('lateral error');grid on
%     subplot(3,2,5)
%     plot(1:NHorizon,error_dis_lin(5,:),'b.',1:NHorizon,error_dis_lin_opt(5,:),'r.')
%     title('steering');grid on
%     subplot(3,2,6)
%     plot(CarStates(1,:),CarStates(2,:),'r.',pre_opt_car_state(1,:),pre_opt_car_state(2,:),'b.')
%     legend('Optimized','Previous Optimization');grid on
   
    
    % Fall back strategy in case optimisation fails:
    % take the following prediction in the last solved optimisation
    if info.exitflag == 1
        iOpt = iOpt + 1;
    else
        iOpt = 2;
        Optimizer_Inputs = U;
    end
    
    % logging
    logging.x_coord(iSim) = car_sim.x;
    logging.y_coord(iSim) = car_sim.y;
    logging.u_opt(iSim,1:NHorizon) = Optimizer_Inputs;
    
    logging.v(iSim) = car_sim.v;
    logging.r(iSim) = car_sim.r;
    logging.d_phi(iSim) = car_sim.d_phi;
    logging.e(iSim) = car_sim.e;
    logging.s(iSim) = car_sim.s;
    logging.a_wheel_angle(iSim) = car_sim.a_wheel_angle;
    logging.v_wheel_angle(iSim) = Optimizer_Inputs(iOpt);

    logging.Fz_fl(iSim) = car_sim.Fz_fl;
    logging.Fz_fr(iSim) = car_sim.Fz_fr;
    logging.Fz_rl(iSim) = car_sim.Fz_rl;
    logging.Fz_rr(iSim) = car_sim.Fz_rr;
    logging.alpha_fl(iSim) = car_sim.alpha_fl;
    logging.alpha_fr(iSim) = car_sim.alpha_fr;
    logging.alpha_rl(iSim) = car_sim.alpha_rl;
    logging.alpha_rr(iSim) = car_sim.alpha_rr;
    logging.force_fl(iSim) = car_sim.force_fl;
    logging.force_fr(iSim) = car_sim.force_fr;
    logging.force_rl(iSim) = car_sim.force_rl;
    logging.force_rr(iSim) = car_sim.force_rr;
    
    logging.a_heading(iSim) = car_sim.a_heading;
    logging.k(iSim) = car_sim.k;
    logging.QPtime(iSim) = info.QPtime;
    logging.Failed(iSim) = info.exitflag;

  
end
toc

%% Plots
Plot_Simulation(logging)

%% Unloading NURBS
car.FreeNURBS();
car_sim.FreeNURBS();


