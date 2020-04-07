classdef Vehicle_v1 < handle
    properties
      Izz = 2380.7;
      mass = 1292.2;
      a = 1.006;
      b = 1.534;
      wheelbase = 1.006+1.534;
      c_f = 70000;
      c_r = 130000;
      delta_t = 0.01;
      x = 0.0;
      y = 0.0;
      a_heading = 0.0;
      u = 10.0;
      v = 0.0;
      r = 0.0;
      s = 0.0;
      k = 0.0;
      e = 0.0;
      d_phi = 0.0;
      a_wheel_angle = 0.0;
      x0 = 0.0;
      y0 = 0.0;
      a_heading0 = 0.0;
      u0 = 10.0;
      v0 = 0.0;
      r0 = 0.0;
      s0 = 0.0;
      k0 = 0.0;
      e0 = 0.0;
      d_phi0 = 0.0;
      a_wheel_angle0 = 0.0;
      dot_u = 0.0;
      dot_v = 0.0;
      dot_r = 0.0;
      dot_s = 0.0;
      dot_e = 0.0;
      dot_d_phi = 0.0;     
      track = [] %track placeholder
      J = []; %Placeholder for Jacobian matrix
    end
    methods
        function obj = Vehicle_v1()
            %Load library
            addpath('C:\Workspaces\MPC\SISL\x64\Debug')
            addpath('C:\Workspaces\MPC\SISL-master\app')
            addpath('C:\Workspaces\MPC\SISL-master\include')
            loadlibrary('SislNurbs.dll','SislNurbs.h')
        end
        function obj = Calculate_states(obj,v_wheel_angle)
            % Slip angles
            alpha_f = -obj.a_wheel_angle + atan2((obj.v + obj.a*obj.r),abs(obj.u));
            alpha_r = atan2(obj.v-obj.b*obj.r,obj.u);

            % Force
            force_f = -obj.c_f.*alpha_f;
            force_r = -obj.c_r.*alpha_r;
            
            % Equations of motion
            % Constant longitudinal speed         
            obj.dot_u = 0;
            obj.dot_v = ((1/obj.mass).* ( force_f.*cos(obj.a_wheel_angle) + force_r)) - obj.r.*obj.u;
            obj.dot_r = (obj.a*force_f.*cos(obj.a_wheel_angle) - obj.b.*force_r)/obj.Izz;
            
            % Euler
            obj.v = obj.v + obj.dot_v * obj.delta_t;
            obj.r = obj.r + obj.dot_r * obj.delta_t;
            
            %Global coordinates
            obj.a_heading = obj.a_heading + obj.r * obj.delta_t;
            obj.x = obj.x + obj.delta_t* (obj.u.*cos(obj.a_heading) - obj.v .* sin(obj.a_heading));
            obj.y = obj.y + obj.delta_t* (obj.u.*sin(obj.a_heading) + obj.v .* cos(obj.a_heading));
            
            %% State vector: v r d_phi e a_wheel_angle

            % d_phi
            a_heading_ref = calllib('SislNurbs','CalculateDerivate',obj.track,obj.s);
            obj.d_phi = obj.a_heading - a_heading_ref;
            param_dist = [0, 0];
            [~,~,~,param_dist] =  calllib('SislNurbs','closestpoint',obj.track, [obj.x, obj.y],param_dist );           
            
            %curvature
            a_heading_K = calllib('SislNurbs','CalculateDerivate',obj.track,obj.s+0.01);
            obj.k = (a_heading_K - a_heading_ref)/0.01;
            
            % Position along trajectory
            obj.s = obj.s + obj.u * cos(obj.d_phi) * obj.delta_t; %param_dist(1);
                                   
           
            % Steering
            obj.a_wheel_angle = obj.a_wheel_angle + obj.delta_t * v_wheel_angle;
            
            %% dot state vector dot_v dot_r dot_d_phi dot_e v_wheel_angle
            %dot_s
            obj.dot_s = obj.u - obj.v * obj.d_phi;
            %dot_e
            obj.dot_e = obj.v + obj.u * obj.d_phi;
            %dot_d_phi
            obj.dot_d_phi = obj.r - obj.k*obj.u;
            
            % Lateral position along tracjectory
            obj.e = obj.e + obj.dot_e * obj.delta_t; %param_dist(2);
            
        end
        
        function obj = CreateTrack(obj,track)
            % Vector form of track
            for i=1:1:666
                vector(2*i-1)= track.center(1,i);
                vector(2*i)=track.center(2,i);
                nptyp(i)=1;
            end
            % libfunctions('SislNurbs')
            % libfunctionsview SislNurbs
            Track_Nurbs = calllib('SislNurbs','createNURBS',vector,nptyp,666);
            obj.track = Track_Nurbs;
        end
        function [long_dist, lat_dist] =  CalculateTrackdistance(obj,x,y)
            param_dist = [0, 0];
            [~,~,~,param_dist] =  calllib('SislNurbs','closestpoint',obj.track, [x, y],param_dist ); 
            long_dist = param_dist(1);
            lat_dist = param_dist(2);
        end
        function phi_track = CalculateTrackPhi(obj,long_dist)
            phi_track = calllib('SislNurbs','CalculateDerivate',obj.track,long_dist);
        end
        function [state_matrix, dot_state_matrix, carstate_matrix] = RunSimulation(obj,tfinal,v_wheel_angle)
            NHorizon = tfinal/obj.delta_t;
            % State vector: v r d_phi e a_wheel_angle
            state_matrix = zeros(5,NHorizon);
            carstate_matrix = zeros(3, NHorizon);
            obj.InitVehicle();
            for i=1:NHorizon
                obj.Calculate_states(v_wheel_angle(i));
                state_matrix(1,i) = obj.v;
                state_matrix(2,i) = obj.r;
                state_matrix(3,i) = obj.d_phi;
                state_matrix(4,i) = obj.e;
                state_matrix(5,i) = obj.a_wheel_angle;
                dot_state_matrix(1,i) = obj.dot_v;
                dot_state_matrix(2,i) = obj.dot_r;
                dot_state_matrix(3,i) = obj.dot_d_phi;
                dot_state_matrix(4,i) = obj.dot_e;
                dot_state_matrix(5,i) = v_wheel_angle(i);
                carstate_matrix(1,i) = obj.x;
                carstate_matrix(2,i) = obj.y;
                carstate_matrix(3,i) = obj.s;
            end
        end
        function obj = InitVehicle(obj)
          obj.x = obj.x0;
          obj.y = obj.y0;
          obj.u = obj.u0;
          obj.v = obj.v0;
          obj.a_heading = obj.a_heading0;
          obj.r = obj.r0;
          obj.a_wheel_angle = obj.a_wheel_angle0;
          obj.s = obj.s0;
          obj.e = obj.e0;
          obj.d_phi = obj.d_phi0;
        end
        function obj = Jacobian_function(obj)
            % returns the discretized, linearized model about (Xbar_k,Ubar_k)
            % s.t. x(k+1) = Ad*x(k) + Bd*u(k) + gd
            % x = [ v r d_phi e a_wheel_angle ]
            % u = v_wheel_angle
            syms v r d_phi e a_wheel_angle v_wheel_angle
            
            a = obj.a;
            b = obj.b;
            c_f = obj.c_f;
            c_r = obj.c_r;
            mass = obj.mass;
            Izz = obj.Izz;
            delta_t = obj.delta_t;
            k = obj.k; %curvature
            u = obj.u;
            
            %slip angles
            syms alpha_f alpha_r  
            alpha_f(v, r, d_phi, e, a_wheel_angle) = -a_wheel_angle + atan2((v + a*r),abs(u));
            alpha_r(v, r, d_phi, e, a_wheel_angle) = atan2(v-b*r,u);

            % Force
            syms force_f force_r
            force_f(v, r, d_phi, e, a_wheel_angle) = -c_f*alpha_f;
            force_r(v, r, d_phi, e, a_wheel_angle) = -c_r*alpha_r;
            
            
            
            %% dot state vector dot_v dot_r dot_d_phi dot_e v_wheel_angle
            % dot_v
            f1(v, r, d_phi, e, a_wheel_angle)= ((1/mass)* ( force_f*cos(a_wheel_angle) + force_r)) - r*u;
            % dot_r
            f2(v, r, d_phi, e, a_wheel_angle)= (a*force_f*cos(a_wheel_angle) - b*force_r)/Izz;
            % dot_d_phi
            f3(v, r, d_phi, e, a_wheel_angle)=  r - k*u;
            % dot_e
            f4(v, r, d_phi, e, a_wheel_angle)= v + u * d_phi;
            % dot_a_steering_wheel = v_steering_wheel
            f5(v, r, d_phi, e, a_wheel_angle)= v_wheel_angle;
            
            
            obj.J = jacobian([f1,f2,f3,f4,f5],[v, r, d_phi, e, a_wheel_angle]);
        end
        function Jacobian_output = Jacobian_eval(obj,x)
            Jacobian_output = double(obj.J(x(1), x(2), x(3), x(4), x(5)));
        end
        function [Ak,Bk,gk] = DiscretizedLinearizedMatrices(obj,dot_x,x,v_wheel_angle)
            % Note this is just for one timestep
            % Vector state x = [v r d_phi e a_wheel_angle]
            % Control state u = v_wheel_angle
            Ac = obj.Jacobian_eval(x);
            Bc = [0; 0; 0; 0; 1];
            f  = [dot_x(1);dot_x(2);dot_x(3);dot_x(4);dot_x(5)];
            
            gc=f-Ac*x-Bc*v_wheel_angle;

            Bc_aug=[Bc gc];

            %discretize
            su=1;%Num control variables 
            sx=5;%Num state variables 
            tmp = expm([Ac Bc_aug; zeros(su+1,sx+su+1)]*obj.delta_t);

            Ad = zeros(sx,sx);
            Bd = zeros(sx,su);
            gd = zeros(sx,1);
            Ak(1:sx,1:sx) =tmp(1:sx,1:sx);
            Bk(1:sx,1:su) =tmp(1:sx,sx+1:sx+su);
            gk(1:sx,1) =tmp(1:sx,sx+su+1);

           
            % following to avoid numerical errors
%             Ad(end,end)=1;
%             Bd(end,end)=obj.delta_t;
        end
    end
end