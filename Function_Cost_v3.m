function [ X,U,info,State_Variables, dot_State_Variables,carstate_matrix, HorizonIter ] = Function_Cost_v3(car,tHorizon,vSteering,x0,u0)


%     Solver = 'CVX';
    Solver = 'QuadProg';

    [State_Variables, dot_State_Variables, carstate_matrix] = car.RunSimulation(tHorizon,vSteering);
    NHorizon = size(State_Variables,2);

    % Pose QP 
    %                min   1/2*x'Hx + x'f
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
        HorizonIter(i).Rk = 5;
        HorizonIter(i).fk = zeros(5,1);%[0;0;-1;-1;0];
                           % v   r d_phi  e   a_wheel_angle v_wheel_angle
        HorizonIter(i).lb = [-5 -3 -1.5  -0.3  -0.5          -0.5]';
        HorizonIter(i).ub = [ 5  3  1.5   0.3   0.5           0.5]';
    end
    HorizonIter(NHorizon-1).Qk = diag([0,0,1,1000,0]);
    HorizonIter(NHorizon).Qk = diag([0,0,0,0,0]);   

    if strcmp(Solver,'CVX')
        [ X,U,info ] = CVXoptimizer(HorizonIter,NHorizon);
    elseif strcmp(Solver,'QuadProg')
        [ X,U,info ] = QuadProgoptimizer_v3(HorizonIter,NHorizon);
    end
    
%     [State_Variables, dot_State_Variables, carstate_matrix] = car.RunSimulation(tHorizon,aSteering);
%     figure(1)
%     plot(carstate_matrix(1,:),carstate_matrix(2,:),'k.')
    

end