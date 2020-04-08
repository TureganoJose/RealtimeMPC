function [ X,U,info ] = Function_Cost_v1(car,tHorizon,vSteering,x0,u0)


    Solver = 'CVX';
    Solver = 'QuadProg';

    [State_Variables, dot_State_Variables, carstate_matrix] = car.RunSimulation(tHorizon,vSteering);
    NHorizon = size(State_Variables,2);
    
    % Pose QP 
    %                min   1/2*x'Hx + x'g
    %                s.t.  lb  <=  x <= ub
    %                      lbA <= Ax <= ubA  {optional}
    %                x = [ v r d_phi e a_wheel_angle ]
    %                u = v_wheel_angle
    
    HorizonIter(1).x0 = x0;
    HorizonIter(1).u0 = u0;
    for i=1:NHorizon
        HorizonIter(i).x = State_Variables(:,i);
        HorizonIter(i).dot_x = dot_State_Variables(:,i);
        HorizonIter(i).u = vSteering(i);
        [HorizonIter(i).Ak,HorizonIter(i).Bk,HorizonIter(i).gk] = car.DiscretizedLinearizedMatrices(HorizonIter(i).dot_x, HorizonIter(i).x,HorizonIter(i).u);
        HorizonIter(i).Qk = diag([0,0,1,1,0]);
        HorizonIter(i).Rk = 0.007;
        HorizonIter(i).fk = zeros(5,1);%[0;0;-1;-1;0];
                           % v   r d_phi  e   a_wheel_angle v_wheel_angle
        HorizonIter(i).lb = [-5 -3 -1.5  -20  -0.5          -0.5]';
        HorizonIter(i).ub = [ 5  3  1.5   20   0.5           0.5]';
    end

    if strcmp(Solver,'CVX')
        [ X,U,info ] = CVXoptimizer(HorizonIter,NHorizon);
    elseif strcmp(Solver,'QuadProg')
        [ X,U,info ] = QuadProgoptimizer(HorizonIter,NHorizon);
    end


end