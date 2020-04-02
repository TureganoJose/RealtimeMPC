classdef Vehicle < handle
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
      a_wheel_angle = 0.0;
      x0 = 0.0;
      y0 = 0.0;
      a_heading0 = 0.0;
      u0 = 10.0;
      v0 = 0.0;
      r0 = 0.0;
      a_wheel_angle0 = 0.0;
      dot_u = 0.0;
      dot_v = 0.0;
      dot_r = 0.0;
      J = []; %Placeholder for Jacobian matrix
    end
    methods
        function obj = Calculate_states(obj,v_wheel_angle)
            % State vector: x y u v a_heading r a_wheel_angle

            % Steering angle
            obj.a_wheel_angle = obj.a_wheel_angle + obj.delta_t * v_wheel_angle;
            
            % Slip angles
            alpha_f = obj.a_wheel_angle - atan2((obj.v + obj.a*obj.r),abs(obj.u));
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
%             speed = sqrt( obj.u^2 + obj.v^2);
%             if(obj.u==0)
%                 beta = 0;
%             else
%                 beta = obj.v/obj.u;
%             end
            obj.a_heading = obj.a_heading + obj.r * obj.delta_t;
            obj.x = obj.x + obj.delta_t* (obj.u.*cos(obj.a_heading) - obj.v .* sin(obj.a_heading));
            obj.y = obj.y + obj.delta_t* (obj.u.*sin(obj.a_heading) + obj.v .* cos(obj.a_heading));

%             obj.x = obj.x + speed * cos(obj.a_heading+beta) * obj.delta_t;
%             obj.y = obj.y + speed * sin(obj.a_heading+beta) * obj.delta_t;
        end
        function state_matrix = RunSimulation(obj,tfinal,v_wheel_angle)
            NHorizon = tfinal/obj.delta_t;
            state_matrix = zeros(6,NHorizon);
            obj.InitVehicle();
            for i=1:NHorizon
                obj.Calculate_states(v_wheel_angle(i));
                state_matrix(1,i) = obj.x;
                state_matrix(2,i) = obj.y;
                state_matrix(3,i) = obj.u;
                state_matrix(4,i) = obj.v;
                state_matrix(5,i) = obj.a_heading;
                state_matrix(6,i) = obj.r;
                state_matrix(7,i) = obj.a_wheel_angle;
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
        end
        function obj = Jacobian_function(obj)
            % returns the discretized, linearized model about (Xbar_k,Ubar_k)
            % s.t. x(k+1) = Ad*x(k) + Bd*u(k) + gd
            % x = [ x y u v a_heading r a_wheel_angle ]
            % u = v_wheel_angle
            syms x y u v a_heading r a_wheel_angle v_wheel_angle

            a = obj.a;
            b = obj.b;
            c_f = obj.c_f;
            c_r = obj.c_r;
            mass = obj.mass;
            Izz = obj.Izz;
            
            
            %slip angles
            syms alpha_f alpha_r  
            alpha_f(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle) = a_wheel_angle - atan2((v + a*r),abs(u));
            alpha_r(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle) = atan2(-v+b*r,u);

            % Force
            syms force_f force_r
            force_f(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle) = c_f*alpha_f;
            force_r(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle) = c_r*alpha_r;
            
            
            %dX
            f1(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle)= u*cos(a_heading) - v * sin(a_heading);
            
            %dY
            f2(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle)= u*sin(a_heading) + v * cos(a_heading);
            
            %du=0;
            f3 = 0;
            
            %dv;
            f4(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle)= ((1/mass)* ( force_f*cos(a_wheel_angle) + force_r)) - r*u;
            
            %da_heading = r
            f5(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle)= r;
            
            %dr
            f6(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle)= (a*force_f*cos(a_wheel_angle) - b*force_r)/Izz;
            
            %da_wheel_angle
            f7(x, y, u, v, a_heading, r, a_wheel_angle, v_wheel_angle) = v_wheel_angle;
            
            
            obj.J = jacobian([f1,f2,f3,f4,f5,f6,f7],[x, y, u, v, a_heading, r, a_wheel_angle]);
        end
        function Jacobian_output = Jacobian_eval(obj,v_wheel_angle)
            Jacobian_output = obj.J(obj.x, obj.y, obj.u, obj.v, obj.a_heading, obj.r, obj.a_wheel_angle);
        end
    end
end