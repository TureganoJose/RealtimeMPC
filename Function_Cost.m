function cost = Function_Cost(car,tHorizon,aSteering,dist_weights,head_weights,traj,track,trackWidth)

    State_Variables = car.RunSimulation(tHorizon,aSteering);
    % compute squared distance to the current location
    TrackX = track.center(1,:);
    TrackY = track.center(2,:);
    
    NHorizon = size(State_Variables,2);
    error_distance = zeros(1, NHorizon);
    error_heading  = zeros(1, NHorizon);
    for i=1:NHorizon
        distanceX = TrackX - State_Variables(1,i);
        distanceY = TrackY - State_Variables(2,i);
        squared_distance_array   = distanceX.^2 + distanceY.^2;
        % find closest point
        [e , minIndex] = min(squared_distance_array);
        error_distance(i) = sqrt(e);
        
        [theta, ~] = findTheta([track.center(1,minIndex),track.center(2,minIndex)],track.center,traj.ppx.breaks,trackWidth,minIndex);
        a_heading_target = atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta));
        error_heading(i) = abs(State_Variables(5,i) - a_heading_target);
    end
    
    cost = sum(dist_weights .* error_distance + head_weights .* error_heading);

end