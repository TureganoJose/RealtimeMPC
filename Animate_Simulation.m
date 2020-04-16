function Animate_Simulation(car,logging)
    
    NHorizon = size(logging.u_opt,2);
    tHorizon = NHorizon*car.delta_t;
    NSim = size(logging.u_opt,1);
    
    e_max = max( logging.e);
    e_min = min( logging.e);
    d_phi_max = max( logging.d_phi);
    d_phi_min = min( logging.d_phi); 
    steering_max = max(logging.a_wheel_angle);
    steering_min = min(logging.a_wheel_angle);
    
	a = car.a;
	b = car.b;
	wheelbase = a + b;
	track_w = car.track_w;
	sqrt_a = sqrt( a^2 + track_w^2);
	sqrt_b = sqrt( b^2 + track_w^2);
	X_box = logging.x_coord + [  sqrt_a.*cos(logging.a_heading + acos(a./sqrt_a)) sqrt_b.*cos(logging.a_heading + pi-acos(b/sqrt_b))    sqrt_b.*cos(logging.a_heading - pi+acos(b/sqrt_b))    sqrt_a*cos(logging.a_heading - acos(a./sqrt_a))         ];
	Y_box = logging.y_coord + [  sqrt_a.*sin(logging.a_heading + acos(a./sqrt_a)) sqrt_b.*sin(logging.a_heading + pi-acos(b/sqrt_b))    sqrt_b.*sin(logging.a_heading - pi+acos(b/sqrt_b))    sqrt_a*sin(logging.a_heading - acos(a./sqrt_a))         ];
    
    
	h = figure('units','normalized','outerposition',[0 0 1 1]);
    filename = 'testAnimated.gif';

    for iSim=1:NSim-NHorizon
        % Initialise prediction
        car.x0 = logging.x_coord(iSim);
        car.y0 = logging.y_coord(iSim);
        %car.u0 = logging.u(iSim);
        car.a_heading0 = logging.a_heading(iSim);
        car.v0 = logging.v(iSim);
        car.r0 = logging.r(iSim);
        car.d_phi0 = logging.d_phi(iSim); 
        car.s0 = logging.s(iSim);
        car.e0 = logging.e(iSim);
        car.a_wheel_angle0 = logging.a_wheel_angle(iSim);
        car.k0 = logging.k;
        
        % Caluclate prediction for each state for optimal input
        [StateVariables, dot_StateVariables, CarStates] =  car.RunSimulation(tHorizon,logging.u_opt(iSim,:));

        % Plot for each timestep
        clf;
        subplot(3,2,[1 3 5]) % Left unique plot
        hold on
        plot(logging.track.outer(1,:),logging.track.outer(2,:),'r')
        plot(logging.track.inner(1,:),logging.track.inner(2,:),'r')
        plot(logging.track.center(1,:),logging.track.center(2,:),'k.')
        plot(CarStates(1,:),CarStates(2,:),'g-*','MarkerSize',1)
        plot(logging.x_coord(iSim),logging.y_coord(iSim),'m.')
        xlim([logging.x_coord(iSim)-30 logging.x_coord(iSim)+30] )
        ylim([logging.y_coord(iSim)-30 logging.y_coord(iSim)+30])

        fill(X_box(iSim,:),Y_box(iSim,:),'r')
        axis equal
        xlabel('X [m]')
        ylabel('Y [m]')
        
        subplot(3,2,2) % Right side plots
        hold on;
        plot( (1:NHorizon)*car.delta_t , StateVariables(5,:), 'g')
        plot( (1:NHorizon)*car.delta_t , logging.a_wheel_angle(iSim:iSim+NHorizon-1), 'm')
        ylim([steering_min steering_max]);
        grid on; 
        legend('Model Prediction','Actual input application')
        xlabel('tHorizon [s]')
        ylabel('steering angle [deg]')
        
        subplot(3,2,4)
        hold on
        plot( (1:NHorizon)*car.delta_t , StateVariables(4,:), 'g')
        plot( (1:NHorizon)*car.delta_t , logging.e(iSim:iSim+NHorizon-1), 'm')
        ylim([e_min e_max]);
        grid on; 
        xlabel('tHorizon [s]')
        ylabel('lateral tracking error [m]')

        subplot(3,2,6)
        hold on
        plot( (1:NHorizon)*car.delta_t , StateVariables(3,:), 'g')
        plot( (1:NHorizon)*car.delta_t , logging.d_phi(iSim:iSim+NHorizon-1), 'm')

        ylim([d_phi_min d_phi_max]);
        grid on; 
        xlabel('tHorizon [s]')
        ylabel('lateral heading angle error [deg]')
        hold off; 
        
        %pause(0.01)
        drawnow 
        % Capture the plot as an image 
        frame = getframe(h); 
        im = frame2im(frame); 
        [imind,cm] = rgb2ind(im,256); 
        % Write to the GIF File 
        if iSim == 1 
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
        else 
            imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append'); 
        end 
    end
end