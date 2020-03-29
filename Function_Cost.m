function cost = Function_Cost(car,tHorizon,vSteering,dist_weights,head_weights,track)

    State_Variables = car.RunSimulation(tHorizon,vSteering);
    % compute squared distance to the current location   
    NHorizon = size(State_Variables,2);
    error_distance = zeros(1, NHorizon);
    error_heading  = zeros(1, NHorizon);
    for i=1:NHorizon
        
        param_dist = [0, 0];
        [~,~,~,param_dist] =  calllib('SislNurbs','closestpoint',track, [State_Variables(1,i), State_Variables(2,i)],param_dist );
        
        error_distance(i) = param_dist(2);
        a_heading_target = calllib('SislNurbs','CalculateDerivate',track,param_dist(1));
        error_heading(i) = abs(State_Variables(5,i) - a_heading_target);
%         figure(4)
%         plot(i,State_Variables(5,i),'b.')
%         hold on
%         plot(i,a_heading_target,'g.')
%         hold on
    end
%     figure(1);hold on;plot(State_Variables(1,:),State_Variables(2,:),'b.')
%     figure(2);hold on;plot(1:NHorizon,error_distance);title('error distance')
%     figure(3);hold on;plot(1:NHorizon,error_heading);title('error heading')

    cost = sum(dist_weights .* error_distance + head_weights .* error_heading);

end