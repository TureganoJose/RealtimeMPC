%% Create double-lane change ISO 3888:1975
clear
B = 1.5; %Car width

track_width = 1.1*B+0.25;

% % Single lane change
% track.center(1,:)= [ 0 10 40 50 65 80 105 118 125 130 150 160];
% track.center(2,:)= [ 1.057 1.057 1.057 2.65 4.1 4.642 4.642 4.642 4.642 4.642 4.642 4.642];

% ISO 3888-1:1999 (Test track for a severe lane-change manoeuvre – Part 1: Double lane-change),
% track.center(1,:)= [ 0 10 25 40 50 65 80 105 118 125 130 140 150 160];
% track.center(2,:)= [1.057 1.057 1.057 1.057 2.65 4.1 4.642 4.642 2.842 1.75 1.057 1.057 1.057 1.057];
% 

% track.center(1,:)= [ 0:1:40 50 65 80 105 118 125 130:1:200];
% track.center(2,:)= [ones(size(0:1:40))*1.057 2.65 4.1 4.642 4.642 2.842 1.75 ones(size(130:1:200))*1.057];
% 
% 
% track.inner(1,:)= track.center(1,:);
% track.inner(2,:)= track.center(2,:)-track_width;
% 
% track.outer(1,:)= track.center(1,:);
% track.outer(2,:)= track.center(2,:)+track_width;


% https://www.researchgate.net/figure/The-schematic-drawing-of-ISO-3888-1-double-lane-change-test-course-see-online-version_fig10_264818560
track_width1 = 1.1*B+0.25;
track_width2 = 1.2*B+0.25;
track_width3 = 1.3*B+0.25;

track.center(1,:)= [ 0:1:15 45:70 95:155];
track.center(2,:)= [ zeros(size(0:1:15)) (3.5-track_width1/2+track_width2/2)*ones(size(45:70)) zeros(size(95:155))];
    
track.inner(1,:)= track.center(1,:);
track.inner(2,:)= [ zeros(size(0:1:15))-track_width1/2 (3.5-track_width1/2)*ones(size(45:70)) zeros(size(95:155))-track_width3/2];

track.outer(1,:)= track.center(1,:);
track.outer(2,:)= [ zeros(size(0:1:15))+track_width1/2 (3.5-track_width1/2+track_width2)*ones(size(45:70)) zeros(size(95:155))+track_width3/2];

hold on
plot(track.center(1,:),track.center(2,:),'r.')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')
hold on
plot(track.outer(1,:),track.outer(2,:),'r')