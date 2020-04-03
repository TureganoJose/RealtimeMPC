classdef Vehicle_v1 < handle
    properties
      Izz = 2380.7;
      mass = 1292.2;
      a = 1.006;
      b = 1.534;
      wheelbase = 1.006+1.534;
      c_f = 70000;
      c_r = 130000;
      delta_t = 0.05;
      x = 0.0;
      y = 0.0;
      a_heading = 0.0;
      u = 10.0;
      v = 0.0;
      r = 0.0;
      s = 0.0;
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
            alpha_f = obj.a_wheel_angle + atan2((obj.v + obj.a*obj.r),abs(obj.u));
            alpha_r = atan2(-obj.v+obj.b*obj.r,obj.u);

            % Force
            force_f = obj.c_f.*alpha_f;
            force_r = obj.c_r.*alpha_r;
            
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
            
            % Position along trajectory
            obj.s = obj.s + obj.u * cos(obj.d_phi) * obj.delta_t; %param_dist(1);
                                   
           
            % Steering
            obj.a_wheel_angle = obj.a_wheel_angle + obj.delta_t * v_wheel_angle;
            
            %% dot state vector dot_v dot_r dot_d_phi v_wheel_angle
            %dot_s
            obj.dot_s = obj.u - obj.v * obj.d_phi;
            %dot_e
            obj.dot_e = obj.v + obj.u * obj.d_phi;
            %dot_d_phi
            k=0; %curvature
            obj.dot_d_phi = obj.r - obj.u;
            
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
        function [state_matrix, carstate_matrix] = RunSimulation(obj,tfinal,v_wheel_angle)
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
%         function obj = Jacobian_function(obj)
%             % returns the discretized, linearized model about (Xbar_k,Ubar_k)
%             % s.t. x(k+1) = Ad*x(k) + Bd*u(k) + gd
%             % x = [ x y u v a_heading r a_wheel_angle ]
%             % u = v_wheel_angle
%             syms x y u v a_heading r a_wheel_angle v_wheel_angle s e V sigma
%             
%             a = obj.a;
%             b = obj.b;
%             c_f = obj.c_f;
%             c_r = obj.c_r;
%             mass = obj.mass;
%             Izz = obj.Izz;
%             delta_t = obj.delta_t;
%             
%             
%             %slip angles
%             syms alpha_f alpha_r  
%             alpha_f(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma) = a_wheel_angle - atan2((v + a*r),abs(u));
%             alpha_r(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma) = atan2(-v+b*r,u);
% 
%             % Force
%             syms force_f force_r
%             force_f(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma) = c_f*alpha_f;
%             force_r(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma) = c_r*alpha_r;
%             
%             
%             %dX
%             f1(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma)= u*cos(a_heading) - v * sin(a_heading);
%             
%             %dY
%             f2(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma)= u*sin(a_heading) + v * cos(a_heading);
%             
%             %du=0;
%             f3 = 0;
%             
%             %dv;
%             f4(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma)= ((1/mass)* ( force_f*cos(a_wheel_angle) + force_r)) - r*u;
%             
%             %da_heading = r
%             f5(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma)= r;
%             
%             %dr
%             f6(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma)= (a*force_f*cos(a_wheel_angle) - b*force_r)/Izz;
%             
%             %da_wheel_angle
%             f7(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle, s, e, V, sigma) = v_wheel_angle;
%             
%           
%             %de
%             f9 = V * cos(sigma);
%             
%             %dsigma
%             f11 = 
%             
%             obj.J = jacobian([f1,f2,f3,f4,f5,f6,f7],[x, y, u, v, a_heading, r, a_wheel_angle]);
%         end
%         function Jacobian_output = Jacobian_eval(obj,v_wheel_angle)
%             Jacobian_output = obj.J(obj.x, obj.y, obj.u, obj.v, obj.a_heading, obj.r, obj.a_wheel_angle);
%         end
    end
end