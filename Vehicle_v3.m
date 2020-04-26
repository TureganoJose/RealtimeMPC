classdef Vehicle_v3 < handle
    properties
      %% Car parameters
      Izz = 2380.7;
      mass = 1292.2;
      a = 1.006;
      b = 1.534;
      wheelbase =  0.0;
      track_w = 0.8;
      cg_height = 0.6;
      c_f = 43730;%70000;
      c_r = 57350;%130000;
      delta_t = 0.05;
            
      %% Car states
      x = 0.0;
      y = 0.0;
      a_heading = 0.0;
      u = 33.333;
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
      u0 = 33.333;
      v0 = 0.0;
      dot_v0 = 0.0;
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
      
      % For logging purposes
      Fz_fl = 0.0;
      Fz_fr = 0.0;
      Fz_rl = 0.0;
      Fz_rr = 0.0;
      alpha_fl = 0.0;
      alpha_fr = 0.0;
      alpha_rl = 0.0;
      alpha_rr = 0.0;
      force_fl = 0.0;
      force_fr = 0.0;
      force_rl = 0.0;
      force_rr = 0.0;
      
      %% Tyre coefficients
      % Front: pac2002_175_70R13.tir
	  f_Fz0 = 3800;
	  f_Pcy1 = 1.4675;
	  f_Pdy1 = 0.94002;
	  f_Pdy2 = -0.17669;
	  f_Pdy3 = -0.69602;       
	  f_Pey1 = 0.0040023;
	  f_Pey2 = 0.00085719;
	  f_Pey3 = 41.465;       
	  f_Pey4 = 665.25;
	  f_Pky1 = -12.536;
	  f_Pky2 = 1.3856; 
	  f_Pky3 = -0.93342;
	  f_Phy1 = 0.0024749;
	  f_Phy2 = 0.0037538;
	  f_Phy3 = 0.037561;
	  f_Pvy1 = 0.031255;
	  f_Pvy2 = -0.0017359;
	  f_Pvy3 = -0.38166;
	  f_Pvy4 = -0.033117;
      
	  f_lambdagy  = 1;
	  f_lambdamuy = 1;
	  f_lambdaCy  = 1;
	  f_lambdaFz0 = 1;
	  f_lambdaKy  = 1;
	  f_lambdaHy  = 1;
	  f_lambdaVy  = 1; 
      f_lambdaEy  = 1;
     
      % Rear: pac2002_195_65R15.tir
      r_Fz0 = 4000;
	  r_Pcy1 = 1.3223;              
      r_Pdy1 = 1.0141;              
      r_Pdy2 = -0.12274;            
      r_Pdy3 = -1.0426;             
      r_Pey1 = -0.63772;           
      r_Pey2 = -0.050782;           
      r_Pey3 = -0.27333;            
      r_Pey4 = -8.3143;             
      r_Pky1 = -19.797;             
      r_Pky2 = 1.7999;              
      r_Pky3 = 0.0095418;           
      r_Phy1 = 0.0011453;           
      r_Phy2 = -6.6688e-005;        
      r_Phy3 = 0.044112;            
      r_Pvy1 = 0.031305;          
      r_Pvy2 = -0.0085749;          
      r_Pvy3 = -0.092912;           
      r_Pvy4 = -0.27907;
      
      r_lambdagy  = 1;
	  r_lambdamuy = 1;
	  r_lambdaCy  = 1;
	  r_lambdaFz0 = 1;
	  r_lambdaKy  = 1;
	  r_lambdaHy  = 1;
	  r_lambdaVy  = 1;
      r_lambdaEy  = 1;

      
      %% Track
      track = [] %track placeholder
      J = []; %Placeholder for Jacobian matrix
    end
    methods
        function obj = Vehicle_v3(obj)
            %Load library
            addpath('C:\Workspaces\MPC\SISL\x64\Debug')
            addpath('C:\Workspaces\MPC\SISL-master\app')
            addpath('C:\Workspaces\MPC\SISL-master\include')
            loadlibrary('SislNurbs.dll','SislNurbs.h')
            obj.wheelbase = obj.a+obj.b;
            obj.Fz_fl = obj.mass * obj.b /(2*(obj.a+obj.b));
            obj.Fz_fr = obj.mass * obj.b /(2*(obj.a+obj.b));
            obj.Fz_rl = obj.mass * obj.a /(2*(obj.a+obj.b));
            obj.Fz_rr = obj.mass * obj.a /(2*(obj.a+obj.b));
            
        end
        function obj = Calculate_states(obj,v_wheel_angle)
            % Slip angles
            obj.alpha_fl = -obj.a_wheel_angle + atan2((obj.v + obj.a*obj.r),obj.u-obj.track_w*obj.r); %
            obj.alpha_fr = -obj.a_wheel_angle + atan2((obj.v + obj.a*obj.r),obj.u+obj.track_w*obj.r); %

            obj.alpha_rl = atan2(obj.v-obj.b*obj.r,obj.u-obj.track_w*obj.r); %
            obj.alpha_rr = atan2(obj.v-obj.b*obj.r,obj.u+obj.track_w*obj.r); %

            % Vertical loads (Note ay = dot_v + u*r) 50% Mech Bal
            obj.Fz_fl = obj.mass*(obj.b*9.81)/(2*(obj.a+obj.b))  -  0.5*obj.mass*(obj.dot_v+obj.r.*obj.u)*obj.cg_height/(2*obj.track_w);
            obj.Fz_fr = obj.mass*(obj.b*9.81)/(2*(obj.a+obj.b))  +  0.5*obj.mass*(obj.dot_v+obj.r.*obj.u)*obj.cg_height/(2*obj.track_w);
            obj.Fz_rl = obj.mass*(obj.a*9.81)/(2*(obj.a+obj.b))  -  0.5*obj.mass*(obj.dot_v+obj.r.*obj.u)*obj.cg_height/(2*obj.track_w);
            obj.Fz_rr = obj.mass*(obj.a*9.81)/(2*(obj.a+obj.b))  +  0.5*obj.mass*(obj.dot_v+obj.r.*obj.u)*obj.cg_height/(2*obj.track_w);

            if(obj.Fz_fl<0) 
                obj.Fz_fl = 1;
            end
            if(obj.Fz_fr<0) 
                obj.Fz_fr = 1;
            end
            if(obj.Fz_rl<0) 
                obj.Fz_rl = 1;
            end
            if(obj.Fz_rr<0) 
                obj.Fz_rr = 1;
            end
            
%             %% Lateral Forces
%             % FL
%             alpha = obj.alpha_fl;
%             Fz = obj.Fz_fl;
%             dfz =(Fz-obj.f_Fz0)/obj.f_Fz0;
%             g = 0.02;
% 
%             obj.force_fl = (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz).* sin( (obj.f_Pcy1 .* obj.f_lambdaCy) * atan( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - ((obj.f_Pey1 + obj.f_Pey2*dfz).* ( 1. - (obj.f_Pey3 + obj.f_Pey4 * g .* obj.f_lambdagy).*sign(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy))) .* obj.f_lambdaEy).* ( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - atan((((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)))))) + (Fz.* ((obj.f_Pvy1 + obj.f_Pvy2 * dfz).*obj.f_lambdaVy + (obj.f_Pvy3 + obj.f_Pvy4*dfz).*g .* obj.f_lambdagy) .* obj.f_lambdamuy);
% 
%             alpha = -obj.alpha_fr;
%             Fz = obj.Fz_fr;
%             dfz =(Fz-obj.f_Fz0)/obj.f_Fz0;
%             g = 0.02;
% 
%             obj.force_fr = -1.*(((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz).* sin( (obj.f_Pcy1 .* obj.f_lambdaCy) * atan( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - ((obj.f_Pey1 + obj.f_Pey2*dfz).* ( 1. - (obj.f_Pey3 + obj.f_Pey4 * g .* obj.f_lambdagy).*sign(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy))) .* obj.f_lambdaEy).* ( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - atan((((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)))))) + (Fz.* ((obj.f_Pvy1 + obj.f_Pvy2 * dfz).*obj.f_lambdaVy + (obj.f_Pvy3 + obj.f_Pvy4*dfz).*g .* obj.f_lambdagy) .* obj.f_lambdamuy);
% 
%             alpha = obj.alpha_rl;
%             Fz = obj.Fz_rl;
%             dfz =(Fz-obj.r_Fz0)/obj.r_Fz0;
%             g = 0.02;
% 
%             obj.force_rl = (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz).* sin( (obj.r_Pcy1 .* obj.r_lambdaCy) * atan( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - ((obj.r_Pey1 + obj.r_Pey2*dfz).* ( 1. - (obj.r_Pey3 + obj.r_Pey4 * g .* obj.r_lambdagy).*sign(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy))) .* obj.r_lambdaEy).* ( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - atan((((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)))))) + (Fz.* ((obj.r_Pvy1 + obj.r_Pvy2 * dfz).*obj.r_lambdaVy + (obj.r_Pvy3 + obj.r_Pvy4*dfz).*g .* obj.r_lambdagy) .* obj.r_lambdamuy);
% 
%             alpha = -obj.alpha_rr;
%             Fz = obj.Fz_rr;
%             dfz =(Fz-obj.f_Fz0)/obj.f_Fz0;
%             g = 0.02;
% 
%             obj.force_rr = -1.*(((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz).* sin( (obj.r_Pcy1 .* obj.r_lambdaCy) * atan( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - ((obj.r_Pey1 + obj.r_Pey2*dfz).* ( 1. - (obj.r_Pey3 + obj.r_Pey4 * g .* obj.r_lambdagy).*sign(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy))) .* obj.r_lambdaEy).* ( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - atan((((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)))))) + (Fz.* ((obj.r_Pvy1 + obj.r_Pvy2 * dfz).*obj.r_lambdaVy + (obj.r_Pvy3 + obj.r_Pvy4*dfz).*g .* obj.r_lambdagy) .* obj.r_lambdamuy);
%                    
            obj.force_fl = -obj.c_f.*obj.alpha_fl;
            obj.force_fr = -obj.c_f.*obj.alpha_fr;
            obj.force_rl = -obj.c_r.*obj.alpha_rl;
            obj.force_rr = -obj.c_r.*obj.alpha_rr;
            
            % Equations of motion
            % Constant longitudinal speed         
            obj.dot_u = 0;
            obj.dot_v = ((1/obj.mass).* ( (obj.force_fl+obj.force_fr).*cos(obj.a_wheel_angle) + (obj.force_rl+obj.force_rr))) - obj.r.*obj.u;
            obj.dot_r = (obj.a*(obj.force_fl+obj.force_fr).*cos(obj.a_wheel_angle) - obj.b.*(obj.force_rl+obj.force_rr))/obj.Izz;
            
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
            obj.d_phi = ang_diff( a_heading_ref,obj.a_heading); % obj.d_phi = obj.a_heading - a_heading_ref;

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
            
            % two points in NURBS
            [long_dist, lat_dist] =  obj.CalculateTrackdistance(obj.x,obj.y);
            position = obj.InterrogateNURBS(long_dist);
            x1 = position(1);
            y1 = position(2);
            x2 = x1 + 0.01*cos(obj.a_heading);
            y2 = y1 + 0.01*sin(obj.a_heading);
            % determine position of point relative to NURBS
            side_pos = sign(  (x2-x1) * (obj.y-y1)  -  (y2-y1) * (obj.x-x1)  );
            obj.e = side_pos * lat_dist;
        end
        function [fy_fl, fy_fr, fy_rl, fy_rr] = CalculateTyreForces(obj,slip_a, gamma, Fz_input)
            % FL
            alpha = slip_a;
            Fz = Fz_input;
            dfz =(Fz-obj.f_Fz0)/obj.f_Fz0;
            g = gamma;

            fy_fl = (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz).* sin( (obj.f_Pcy1 .* obj.f_lambdaCy) * atan( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - ((obj.f_Pey1 + obj.f_Pey2*dfz).* ( 1. - (obj.f_Pey3 + obj.f_Pey4 * g .* obj.f_lambdagy).*sign(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy))) .* obj.f_lambdaEy).* ( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - atan((((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)))))) + (Fz.* ((obj.f_Pvy1 + obj.f_Pvy2 * dfz).*obj.f_lambdaVy + (obj.f_Pvy3 + obj.f_Pvy4*dfz).*g .* obj.f_lambdagy) .* obj.f_lambdamuy);

            alpha = slip_a;
            Fz = Fz_input;
            dfz =(Fz-obj.f_Fz0)/obj.f_Fz0;
            g = gamma;

            fy_fr = (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz).* sin( (obj.f_Pcy1 .* obj.f_lambdaCy) * atan( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - ((obj.f_Pey1 + obj.f_Pey2*dfz).* ( 1. - (obj.f_Pey3 + obj.f_Pey4 * g .* obj.f_lambdagy).*sign(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy))) .* obj.f_lambdaEy).* ( (((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)) - atan((((obj.f_Pky1*obj.f_Fz0 * sin(2. * atan(Fz/(obj.f_Pky2*obj.f_Fz0)))).* (1. - obj.f_Pky3 * abs(g .* obj.f_lambdagy)) .* obj.f_lambdaFz0 .* obj.f_lambdaKy)./ ((obj.f_Pcy1 .* obj.f_lambdaCy) * (((obj.f_Pdy1 + obj.f_Pdy2*dfz).* (1.- obj.f_Pdy3 * (g .* obj.f_lambdagy).^2) .* obj.f_lambdamuy).* Fz))).*(alpha + ((obj.f_Phy1 + obj.f_Phy2 * dfz).*obj.f_lambdaHy + obj.f_Phy3 * g .* obj.f_lambdagy)))))) + (Fz.* ((obj.f_Pvy1 + obj.f_Pvy2 * dfz).*obj.f_lambdaVy + (obj.f_Pvy3 + obj.f_Pvy4*dfz).*g .* obj.f_lambdagy) .* obj.f_lambdamuy);

            alpha = slip_a;
            Fz = Fz_input;
            dfz =(Fz-obj.r_Fz0)/obj.r_Fz0;
            g = gamma;

            fy_rl = (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz).* sin( (obj.r_Pcy1 .* obj.r_lambdaCy) * atan( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - ((obj.r_Pey1 + obj.r_Pey2*dfz).* ( 1. - (obj.r_Pey3 + obj.r_Pey4 * g .* obj.r_lambdagy).*sign(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy))) .* obj.r_lambdaEy).* ( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - atan((((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)))))) + (Fz.* ((obj.r_Pvy1 + obj.r_Pvy2 * dfz).*obj.r_lambdaVy + (obj.r_Pvy3 + obj.r_Pvy4*dfz).*g .* obj.r_lambdagy) .* obj.r_lambdamuy);

            alpha = slip_a;
            Fz = Fz_input;
            dfz =(Fz-obj.f_Fz0)/obj.f_Fz0;
            g = gamma;

            fy_rr = (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz).* sin( (obj.r_Pcy1 .* obj.r_lambdaCy) * atan( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - ((obj.r_Pey1 + obj.r_Pey2*dfz).* ( 1. - (obj.r_Pey3 + obj.r_Pey4 * g .* obj.r_lambdagy).*sign(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy))) .* obj.r_lambdaEy).* ( (((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)) - atan((((obj.r_Pky1*obj.r_Fz0 * sin(2. * atan(Fz/(obj.r_Pky2*obj.r_Fz0)))).* (1. - obj.r_Pky3 * abs(g .* obj.r_lambdagy)) .* obj.r_lambdaFz0 .* obj.r_lambdaKy)./ ((obj.r_Pcy1 .* obj.r_lambdaCy) * (((obj.r_Pdy1 + obj.r_Pdy2*dfz).* (1.- obj.r_Pdy3 * (g .* obj.r_lambdagy).^2) .* obj.r_lambdamuy).* Fz))).*(alpha + ((obj.r_Phy1 + obj.r_Phy2 * dfz).*obj.r_lambdaHy + obj.r_Phy3 * g .* obj.r_lambdagy)))))) + (Fz.* ((obj.r_Pvy1 + obj.r_Pvy2 * dfz).*obj.r_lambdaVy + (obj.r_Pvy3 + obj.r_Pvy4*dfz).*g .* obj.r_lambdagy) .* obj.r_lambdamuy);
            
        end
        function obj = CreateTrack(obj,track)
            NTrack = size(track.center  ,2);
            % Vector form of track
            for i=1:1:NTrack
                vector(2*i-1)= track.center(1,i);
                vector(2*i)=track.center(2,i);
                nptyp(i)=1;
            end
            % libfunctions('SislNurbs')
            % libfunctionsview SislNurbs
            Track_Nurbs = calllib('SislNurbs','createNURBS',vector,nptyp,NTrack);
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
        function position = InterrogateNURBS(obj,long_param)
            position = [0, 0];
            [~,~,position]=calllib('SislNurbs','interrogateNURBS',obj.track,long_param,position);
        end
        function FreeNURBS(obj)
            calllib('SislNurbs','freeNURBS',obj.track)
            unloadlibrary SislNurbs
        end
        function [state_matrix, dot_state_matrix, carstate_matrix] = RunSimulation(obj,tfinal,v_wheel_angle)
            NHorizon = tfinal/obj.delta_t;
            % State vector: v r d_phi e a_wheel_angle
            state_matrix = zeros(5,NHorizon);
            carstate_matrix = zeros(3, NHorizon);
            dot_state_matrix = zeros(5,NHorizon);
            
            obj.InitVehicle();

            state_matrix(1,1) = obj.v;
            state_matrix(2,1) = obj.r;
            state_matrix(3,1) = obj.d_phi;
            state_matrix(4,1) = obj.e;
            state_matrix(5,1) = obj.a_wheel_angle;
            dot_state_matrix(1,1) = obj.dot_v;
            dot_state_matrix(2,1) = obj.dot_r;
            dot_state_matrix(3,1) = obj.dot_d_phi;
            dot_state_matrix(4,1) = obj.dot_e;
            dot_state_matrix(5,1) = v_wheel_angle(1);
            carstate_matrix(1,1) = obj.x;
            carstate_matrix(2,1) = obj.y;
            carstate_matrix(3,1) = obj.s;
            carstate_matrix(4,1) = obj.Fz_fl;
            carstate_matrix(5,1) = obj.Fz_fr;
            carstate_matrix(6,1) = obj.Fz_rl;
            carstate_matrix(7,1) = obj.Fz_rr;
            carstate_matrix(8,1) = obj.alpha_fl;
            carstate_matrix(9,1) = obj.alpha_fr;
            carstate_matrix(10,1) = obj.alpha_rl;
            carstate_matrix(11,1) = obj.alpha_rr;
            carstate_matrix(12,1) = obj.force_fl;
            carstate_matrix(13,1) = obj.force_fr;
            carstate_matrix(14,1) = obj.force_rl;
            carstate_matrix(15,1) = obj.force_rr;

            for i=2:NHorizon
                obj.Calculate_states(v_wheel_angle(i-1));
                state_matrix(1,i) = obj.v;
                state_matrix(2,i) = obj.r;
                state_matrix(3,i) = obj.d_phi;
                state_matrix(4,i) = obj.e;
                state_matrix(5,i) = obj.a_wheel_angle;
                dot_state_matrix(1,i) = obj.dot_v;
                dot_state_matrix(2,i) = obj.dot_r;
                dot_state_matrix(3,i) = obj.dot_d_phi;
                dot_state_matrix(4,i) = obj.dot_e;
                dot_state_matrix(5,i) = v_wheel_angle(i-1);
                carstate_matrix(1,i) = obj.x;
                carstate_matrix(2,i) = obj.y;
                carstate_matrix(3,i) = obj.s;
                carstate_matrix(4,i) = obj.Fz_fl;
                carstate_matrix(5,i) = obj.Fz_fr;
                carstate_matrix(6,i) = obj.Fz_rl;
                carstate_matrix(7,i) = obj.Fz_rr;
                carstate_matrix(8,i) = obj.alpha_fl;
                carstate_matrix(9,i) = obj.alpha_fr;
                carstate_matrix(10,i) = obj.alpha_rl;
                carstate_matrix(11,i) = obj.alpha_rr;
                carstate_matrix(12,i) = obj.force_fl;
                carstate_matrix(13,i) = obj.force_fr;
                carstate_matrix(14,i) = obj.force_rl;
                carstate_matrix(15,i) = obj.force_rr;               
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
            % lateral acceleration as an input to calculate weight transfer
            syms v r d_phi e a_wheel_angle v_wheel_angle
            
            a = obj.a;
            b = obj.b;
            track_w = obj.track_w;
            cg_height = obj.cg_height;
            c_f = obj.c_f;
            c_r = obj.c_r;
            mass = obj.mass;
            Izz = obj.Izz;
            delta_t = obj.delta_t;
            k = obj.k; %curvature
            u = obj.u;
            
		    % Front: pac2002_175_70R13.tir
		    f_Fz0 =    obj.f_Fz0;
		    f_Pcy1 =   obj.f_Pcy1;
		    f_Pdy1 =   obj.f_Pdy1;
		    f_Pdy2 =   obj.f_Pdy2;
		    f_Pdy3 =   obj.f_Pdy3;  
		    f_Pey1 =   obj.f_Pey1;
		    f_Pey2 =   obj.f_Pey2;
		    f_Pey3 =   obj.f_Pey3;
		    f_Pey4 =   obj.f_Pey4;
		    f_Pky1 =   obj.f_Pky1;
		    f_Pky2 =   obj.f_Pky2;
		    f_Pky3 =   obj.f_Pky3;
		    f_Phy1 =   obj.f_Phy1;
		    f_Phy2 =   obj.f_Phy2;
		    f_Phy3 =   obj.f_Phy3;
		    f_Pvy1 =   obj.f_Pvy1;
		    f_Pvy2 =   obj.f_Pvy2;
		    f_Pvy3 =   obj.f_Pvy3;
		    f_Pvy4 =   obj.f_Pvy4;
		    
		    f_lambdagy  = obj.f_lambdagy;
		    f_lambdamuy = obj.f_lambdamuy;
		    f_lambdaCy  = obj.f_lambdaCy;
		    f_lambdaFz0 = obj.f_lambdaFz0;
		    f_lambdaKy  = obj.f_lambdaKy;
		    f_lambdaHy  = obj.f_lambdaHy;
		    f_lambdaVy  = obj.f_lambdaVy;
		    f_lambdaEy  = obj.f_lambdaEy;
		    
		    % Rear: pac2002_195_65R15.tir
		    r_Fz0  =  obj.r_Fz0;
		    r_Pcy1 =  obj.r_Pcy1;       
		    r_Pdy1 =  obj.r_Pdy1;       
		    r_Pdy2 =  obj.r_Pdy2;       
		    r_Pdy3 =  obj.r_Pdy3;       
		    r_Pey1 =  obj.r_Pey1;      
		    r_Pey2 =  obj.r_Pey2;       
		    r_Pey3 =  obj.r_Pey3;       
		    r_Pey4 =  obj.r_Pey4;       
		    r_Pky1 =  obj.r_Pky1;       
		    r_Pky2 =  obj.r_Pky2;       
		    r_Pky3 =  obj.r_Pky3;       
		    r_Phy1 =  obj.r_Phy1;       
		    r_Phy2 =  obj.r_Phy2;       
		    r_Phy3 =  obj.r_Phy3;       
		    r_Pvy1 =  obj.r_Pvy1;     
		    r_Pvy2 =  obj.r_Pvy2;       
		    r_Pvy3 =  obj.r_Pvy3;       
		    r_Pvy4 =  obj.r_Pvy4;
		    
		    r_lambdagy  = obj.r_lambdagy;
		    r_lambdamuy = obj.r_lambdamuy;
		    r_lambdaCy  = obj.r_lambdaCy;
		    r_lambdaFz0 = obj.r_lambdaFz0;
		    r_lambdaKy  = obj.r_lambdaKy;
		    r_lambdaHy  = obj.r_lambdaHy;
		    r_lambdaVy  = obj.r_lambdaVy;
		    r_lambdaEy  = obj.r_lambdaEy;
            
            % Slip angles
            syms alpha_fl alpha_fr alpha_rl alpha_rr 
            
            alpha_fl(v, r, d_phi, e, a_wheel_angle)  = -a_wheel_angle + atan2((v + a*r),u-track_w*r); %
            alpha_fr(v, r, d_phi, e, a_wheel_angle)  = -a_wheel_angle + atan2((v + a*r),u+track_w*r); %

            alpha_rl(v, r, d_phi, e, a_wheel_angle)  = atan2(v-b*r,u-track_w*r); %
            alpha_rr(v, r, d_phi, e, a_wheel_angle)  = atan2(v-b*r,u+track_w*r); %
            
            % Vertical loads (Note ay = dot_v + u*r) and dot_v = f1
            syms Fz_fl Fz_fr Fz_rl Fz_rr
            syms dot_v
            
            Fz_fl(v, r, d_phi, e, a_wheel_angle) = mass*(b*9.81)/(2*(a+b))  -  0.5*mass*(dot_v+r.*u)*cg_height/(2*track_w);
            Fz_fr(v, r, d_phi, e, a_wheel_angle) = mass*(b*9.81)/(2*(a+b))  +  0.5*mass*(dot_v+r.*u)*cg_height/(2*track_w);
            Fz_rl(v, r, d_phi, e, a_wheel_angle) = mass*(a*9.81)/(2*(a+b))  -  0.5*mass*(dot_v+r.*u)*cg_height/(2*track_w);
            Fz_rr(v, r, d_phi, e, a_wheel_angle) = mass*(a*9.81)/(2*(a+b))  +  0.5*mass*(dot_v+r.*u)*cg_height/(2*track_w);
            
                        
%             % Force
%             syms force_fl force_fr force_rl force_rr
%             
%             dFz_fl(v, r, d_phi, e, a_wheel_angle) =(Fz_fl-f_Fz0)/f_Fz0;
%             g = 0.02;
% 
%             force_fl(v, r, d_phi, e, a_wheel_angle) = (((f_Pdy1 + f_Pdy2*dFz_fl).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fl).* sin( (f_Pcy1 .* f_lambdaCy) * atan( (((f_Pky1*f_Fz0 * sin(2. * atan(Fz_fl/(f_Pky2*f_Fz0)))).* (1. - f_Pky3 * abs(g .* f_lambdagy)) .* f_lambdaFz0 .* f_lambdaKy)./ ((f_Pcy1 .* f_lambdaCy) * (((f_Pdy1 + f_Pdy2*dFz_fl).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fl))).*(alpha_fl + ((f_Phy1 + f_Phy2 * dFz_fl).*f_lambdaHy + f_Phy3 * g .* f_lambdagy)) - ((f_Pey1 + f_Pey2*dFz_fl).* ( 1. - (f_Pey3 + f_Pey4 * g .* f_lambdagy).*sign(alpha_fl + ((f_Phy1 + f_Phy2 * dFz_fl).*f_lambdaHy + f_Phy3 * g .* f_lambdagy))) .* f_lambdaEy).* ( (((f_Pky1*f_Fz0 * sin(2. * atan(Fz_fl/(f_Pky2*f_Fz0)))).* (1. - f_Pky3 * abs(g .* f_lambdagy)) .* f_lambdaFz0 .* f_lambdaKy)./ ((f_Pcy1 .* f_lambdaCy) * (((f_Pdy1 + f_Pdy2*dFz_fl).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fl))).*(alpha_fl + ((f_Phy1 + f_Phy2 * dFz_fl).*f_lambdaHy + f_Phy3 * g .* f_lambdagy)) - atan((((f_Pky1*f_Fz0 * sin(2. * atan(Fz_fl/(f_Pky2*f_Fz0)))).* (1. - f_Pky3 * abs(g .* f_lambdagy)) .* f_lambdaFz0 .* f_lambdaKy)./ ((f_Pcy1 .* f_lambdaCy) * (((f_Pdy1 + f_Pdy2*dFz_fl).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fl))).*(alpha_fl + ((f_Phy1 + f_Phy2 * dFz_fl).*f_lambdaHy + f_Phy3 * g .* f_lambdagy)))))) + (Fz_fl.* ((f_Pvy1 + f_Pvy2 * dFz_fl).*f_lambdaVy + (f_Pvy3 + f_Pvy4*dFz_fl).*g .* f_lambdagy) .* f_lambdamuy);
% 
%             dFz_fr(v, r, d_phi, e, a_wheel_angle) =(Fz_fr-f_Fz0)/f_Fz0;
%             g = 0.02;
% 
%             force_fr(v, r, d_phi, e, a_wheel_angle) = -1.*(((f_Pdy1 + f_Pdy2*dFz_fr).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fr).* sin( (f_Pcy1 .* f_lambdaCy) * atan( (((f_Pky1*f_Fz0 * sin(2. * atan(Fz_fr/(f_Pky2*f_Fz0)))).* (1. - f_Pky3 * abs(g .* f_lambdagy)) .* f_lambdaFz0 .* f_lambdaKy)./ ((f_Pcy1 .* f_lambdaCy) * (((f_Pdy1 + f_Pdy2*dFz_fr).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fr))).*(-alpha_fr + ((f_Phy1 + f_Phy2 * dFz_fr).*f_lambdaHy + f_Phy3 * g .* f_lambdagy)) - ((f_Pey1 + f_Pey2*dFz_fr).* ( 1. - (f_Pey3 + f_Pey4 * g .* f_lambdagy).*sign(-alpha_fr + ((f_Phy1 + f_Phy2 * dFz_fr).*f_lambdaHy + f_Phy3 * g .* f_lambdagy))) .* f_lambdaEy).* ( (((f_Pky1*f_Fz0 * sin(2. * atan(Fz_fr/(f_Pky2*f_Fz0)))).* (1. - f_Pky3 * abs(g .* f_lambdagy)) .* f_lambdaFz0 .* f_lambdaKy)./ ((f_Pcy1 .* f_lambdaCy) * (((f_Pdy1 + f_Pdy2*dFz_fr).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fr))).*(-alpha_fr + ((f_Phy1 + f_Phy2 * dFz_fr).*f_lambdaHy + f_Phy3 * g .* f_lambdagy)) - atan((((f_Pky1*f_Fz0 * sin(2. * atan(Fz_fr/(f_Pky2*f_Fz0)))).* (1. - f_Pky3 * abs(g .* f_lambdagy)) .* f_lambdaFz0 .* f_lambdaKy)./ ((f_Pcy1 .* f_lambdaCy) * (((f_Pdy1 + f_Pdy2*dFz_fr).* (1.- f_Pdy3 * (g .* f_lambdagy).^2) .* f_lambdamuy).* Fz_fr))).*(-alpha_fr + ((f_Phy1 + f_Phy2 * dFz_fr).*f_lambdaHy + f_Phy3 * g .* f_lambdagy)))))) + (Fz_fr.* ((f_Pvy1 + f_Pvy2 * dFz_fr).*f_lambdaVy + (f_Pvy3 + f_Pvy4*dFz_fr).*g .* f_lambdagy) .* f_lambdamuy);
% 
%             dFz_rl(v, r, d_phi, e, a_wheel_angle) =(Fz_rl-r_Fz0)/r_Fz0;
%             g = 0.02;
% 
%             force_rl(v, r, d_phi, e, a_wheel_angle) = (((r_Pdy1 + r_Pdy2*dFz_rl).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rl).* sin( (r_Pcy1 .* r_lambdaCy) * atan( (((r_Pky1*r_Fz0 * sin(2. * atan(Fz_rl/(r_Pky2*r_Fz0)))).* (1. - r_Pky3 * abs(g .* r_lambdagy)) .* r_lambdaFz0 .* r_lambdaKy)./ ((r_Pcy1 .* r_lambdaCy) * (((r_Pdy1 + r_Pdy2*dFz_rl).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rl))).*(alpha_rl + ((r_Phy1 + r_Phy2 * dFz_rl).*r_lambdaHy + r_Phy3 * g .* r_lambdagy)) - ((r_Pey1 + r_Pey2*dFz_rl).* ( 1. - (r_Pey3 + r_Pey4 * g .* r_lambdagy).*sign(alpha_rl + ((r_Phy1 + r_Phy2 * dFz_rl).*r_lambdaHy + r_Phy3 * g .* r_lambdagy))) .* r_lambdaEy).* ( (((r_Pky1*r_Fz0 * sin(2. * atan(Fz_rl/(r_Pky2*r_Fz0)))).* (1. - r_Pky3 * abs(g .* r_lambdagy)) .* r_lambdaFz0 .* r_lambdaKy)./ ((r_Pcy1 .* r_lambdaCy) * (((r_Pdy1 + r_Pdy2*dFz_rl).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rl))).*(alpha_rl + ((r_Phy1 + r_Phy2 * dFz_rl).*r_lambdaHy + r_Phy3 * g .* r_lambdagy)) - atan((((r_Pky1*r_Fz0 * sin(2. * atan(Fz_rl/(r_Pky2*r_Fz0)))).* (1. - r_Pky3 * abs(g .* r_lambdagy)) .* r_lambdaFz0 .* r_lambdaKy)./ ((r_Pcy1 .* r_lambdaCy) * (((r_Pdy1 + r_Pdy2*dFz_rl).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rl))).*(alpha_rl + ((r_Phy1 + r_Phy2 * dFz_rl).*r_lambdaHy + r_Phy3 * g .* r_lambdagy)))))) + (Fz_rl.* ((r_Pvy1 + r_Pvy2 * dFz_rl).*r_lambdaVy + (r_Pvy3 + r_Pvy4*dFz_rl).*g .* r_lambdagy) .* r_lambdamuy);
% 
%             dFz_rr(v, r, d_phi, e, a_wheel_angle) =(Fz_rr-r_Fz0)/r_Fz0;
%             g = 0.02;
% 
%             force_rr(v, r, d_phi, e, a_wheel_angle) = -1.*(((r_Pdy1 + r_Pdy2*dFz_rr).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rr).* sin( (r_Pcy1 .* r_lambdaCy) * atan( (((r_Pky1*r_Fz0 * sin(2. * atan(Fz_rr/(r_Pky2*r_Fz0)))).* (1. - r_Pky3 * abs(g .* r_lambdagy)) .* r_lambdaFz0 .* r_lambdaKy)./ ((r_Pcy1 .* r_lambdaCy) * (((r_Pdy1 + r_Pdy2*dFz_rr).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rr))).*(-alpha_rr + ((r_Phy1 + r_Phy2 * dFz_rr).*r_lambdaHy + r_Phy3 * g .* r_lambdagy)) - ((r_Pey1 + r_Pey2*dFz_rr).* ( 1. - (r_Pey3 + r_Pey4 * g .* r_lambdagy).*sign(-alpha_rr + ((r_Phy1 + r_Phy2 * dFz_rr).*r_lambdaHy + r_Phy3 * g .* r_lambdagy))) .* r_lambdaEy).* ( (((r_Pky1*r_Fz0 * sin(2. * atan(Fz_rr/(r_Pky2*r_Fz0)))).* (1. - r_Pky3 * abs(g .* r_lambdagy)) .* r_lambdaFz0 .* r_lambdaKy)./ ((r_Pcy1 .* r_lambdaCy) * (((r_Pdy1 + r_Pdy2*dFz_rr).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rr))).*(-alpha_rr + ((r_Phy1 + r_Phy2 * dFz_rr).*r_lambdaHy + r_Phy3 * g .* r_lambdagy)) - atan((((r_Pky1*r_Fz0 * sin(2. * atan(Fz_rr/(r_Pky2*r_Fz0)))).* (1. - r_Pky3 * abs(g .* r_lambdagy)) .* r_lambdaFz0 .* r_lambdaKy)./ ((r_Pcy1 .* r_lambdaCy) * (((r_Pdy1 + r_Pdy2*dFz_rr).* (1.- r_Pdy3 * (g .* r_lambdagy).^2) .* r_lambdamuy).* Fz_rr))).*(-alpha_rr + ((r_Phy1 + r_Phy2 * dFz_rr).*r_lambdaHy + r_Phy3 * g .* r_lambdagy)))))) + (Fz_rr.* ((r_Pvy1 + r_Pvy2 * dFz_rr).*r_lambdaVy + (r_Pvy3 + r_Pvy4*dFz_rr).*g .* r_lambdagy) .* r_lambdamuy);

            force_fl(v, r, d_phi, e, a_wheel_angle) = -c_f.*alpha_fl;
            force_fr(v, r, d_phi, e, a_wheel_angle) = -c_f.*alpha_fr;
            force_rl(v, r, d_phi, e, a_wheel_angle) = -c_r.*alpha_rl;
            force_rr(v, r, d_phi, e, a_wheel_angle) = -c_r.*alpha_rr;
            
            
            %% dot state vector dot_v dot_r dot_d_phi dot_e v_wheel_angle
            % dot_v
            f1(v, r, d_phi, e, a_wheel_angle)= ((1/mass)* ( (force_fl+force_fr)*cos(a_wheel_angle) + (force_rl+force_rr))) - r*u;
            % dot_r
            f2(v, r, d_phi, e, a_wheel_angle)= (a*(force_fl+force_fr)*cos(a_wheel_angle) - b*(force_rl+force_rr))/Izz;
            % dot_d_phi
            f3(v, r, d_phi, e, a_wheel_angle)=  r - k*u;
            % dot_e
            f4(v, r, d_phi, e, a_wheel_angle)= v + u * d_phi;
            % dot_a_steering_wheel = v_steering_wheel
            f5(v, r, d_phi, e, a_wheel_angle)= v_wheel_angle;
            
%             % Weights
%             syms q_e q_phi q_R
%             % e
%             f6(v, r, d_phi, e, a_wheel_angle) = e + (v+u*d_phi)*delta_t;
%             % d_phi
%             f7(v, r, d_phi, e, a_wheel_angle) = d_phi + (r - k*u)*delta_t;
%             
%             cost(v, r, d_phi, e, a_wheel_angle)= q_e*(f6^2) + q_phi*(f7^2) + q_R*(v_wheel_angle^2);
%             J_cost = jacobian(cost,[v, r, d_phi, e, a_wheel_angle, v_wheel_angle]);
%             H_cost = hessian(cost,[v, r, d_phi, e, a_wheel_angle, v_wheel_angle]);

            obj.J = jacobian([f1,f2,f3,f4,f5],[v, r, d_phi, e, a_wheel_angle]);
        end
        function Jacobian_output = Jacobian_eval(obj,dot_x,x)
%             Jacobian_output = double(obj.J(x(1), x(2), x(3), x(4), x(5)));
%             matlabFunction(car.J,'File','CalculateJ');
            v = x(1);
            r = x(2);
            d_phi = x(3);
            e = x(4);
            a_wheel_angle = x(5);
            dot_v = dot_x(1);
%             Jacobian_output = CalculateJ(v,r,d_phi,e,a_wheel_angle,dot_v);
            Jacobian_output = CalculateJ_linear(v,r,d_phi,e,a_wheel_angle);
            
        
        end
        function [Ak,Bk,gk] = DiscretizedLinearizedMatrices(obj,dot_x,x,v_wheel_angle)
            % Note this is just for one timestep
            % Vector state x = [v r d_phi e a_wheel_angle]
            % Control state u = v_wheel_angle
            Ac = obj.Jacobian_eval(dot_x,x);
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
            %gk(sx,1) = 0.0;
            
            % Alternatively use c2d(sys,Ts)
            % sys = ss(A,B,C,D,ts)
           
            % following to avoid numerical errors
%             Ad(end,end)=1;
%             Bd(end,end)=obj.delta_t;
        end
    end
end