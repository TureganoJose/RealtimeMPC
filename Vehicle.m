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
      x0 = 0.0;
      y0 = 0.0;
      a_heading0 = 0.0;
      u0 = 10.0;
      v0 = 0.0;
      r0 = 0.0;
      dot_u = 0.0;
      dot_v = 0.0;
      dot_r = 0.0;
    end
    methods
        function obj = Calculate_states(obj,a_wheel_angle)
            % State vector: x y u v a_heading r 

            % Slip angles
            alpha_f = a_wheel_angle - atan2((obj.v + obj.a*obj.r),abs(obj.u));
            alpha_r = atan2(-obj.v+obj.b*obj.r,obj.u);

            % Force
            force_f = obj.c_f.*alpha_f;
            force_r = obj.c_r.*alpha_r;
            % Equations of motion
            % Constant longitudinal speed
                       
            obj.dot_u = 0;
            obj.dot_v = ((1/obj.mass).* ( force_f.*cos(a_wheel_angle) + force_r)) - obj.r.*obj.u;
            obj.dot_r = (obj.a*force_f.*cos(a_wheel_angle) - obj.b.*force_r)/obj.Izz;
            
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
        function state_matrix = RunSimulation(obj,tfinal,a_wheel_angle)
            NHorizon = tfinal/obj.delta_t;
            state_matrix = zeros(6,NHorizon);
            obj.InitVehicle();
            for i=1:NHorizon
                obj.Calculate_states(a_wheel_angle(i));
                state_matrix(1,i) = obj.x;
                state_matrix(2,i) = obj.y;
                state_matrix(3,i) = obj.u;
                state_matrix(4,i) = obj.v;
                state_matrix(5,i) = obj.a_heading;
                state_matrix(6,i) = obj.r;
            end
        end
        function obj = InitVehicle(obj)
          obj.x = obj.x0;
          obj.y = obj.y0;
          obj.a_heading = obj.a_heading0;
          obj.u = obj.u0;
          obj.v = obj.v0;
          obj.r = obj.r0;
        end
    end
end