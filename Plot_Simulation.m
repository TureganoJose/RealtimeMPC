function Plot_Simulation(logging)

    figure(1);
    plot(logging.track.outer(1,:),logging.track.outer(2,:),'r')
    hold on
    plot(logging.track.inner(1,:),logging.track.inner(2,:),'r')
    hold on
    plot(logging.track.center(1,:),logging.track.center(2,:),'k.')
    hold on

    figure(1)
    plot( logging.x_coord,logging.y_coord,'b.')
    %axis equal
    hold on
    
    figure(2)
    subplot(3,2,1)
    plot(logging.time,logging.v,'b.')
    grid on; title('v vs time')
    subplot(3,2,2)
    plot(logging.time,logging.r,'b.')
    grid on; title('yaw rate vs time')
    subplot(3,2,3)
    plot(logging.time,logging.d_phi,'b.')
    grid on; title('d_phi vs time')
    subplot(3,2,4)
    plot(logging.time,logging.e,'b.')
    grid on; title('lateral error vs time')
    subplot(3,2,5)
    plot(logging.time,logging.a_wheel_angle,'b.')
    grid on; title('Wheel_angle vs time')
    subplot(3,2,6)
    plot(logging.time,logging.s,'b.')
    grid on; title('s param');
    
    figure(3)
    subplot(2,2,1)
    plot(logging.time, logging.k,'b.')
    grid on; title('Curvature vs time')
    subplot(2,2,2)
    plot(logging.time,logging.a_heading,'b.')
    grid on; title('car Heading angle vs time')
    subplot(2,2,3)
    plot(logging.Failed,'b.')
    grid on; title('Failed Opt')
    subplot(2,2,4)
    plot(logging.QPtime,'b')
    grid on; title('QP solving time (s)')
end