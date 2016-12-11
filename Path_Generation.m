%% ME C231A Project: Urban Driving
% Main execution file
% Group

function [curve_x, curve_y, v , curve_psi ] = Path_Generation()
%% Dimensions (all sizes in meters)
lane_width = 3;

car_width = 1.85;
car_length = 4.9;

turn_radius = 5.56;

%% Path generation
% points for a right hand turn
travel = 5;
waypoints = [[0; 0], [0; travel - car_length/2], [0; travel + lane_width/2], [lane_width/2 + car_length/2; travel + lane_width/2], [lane_width/2 + travel; travel + lane_width/2]];

lane_lower = [[0 + lane_width/2; 0], [0 + lane_width/2; travel], [0 + lane_width/2; travel], [lane_width/2; travel], [lane_width/2 + travel; travel]];
lane_upper = [[0 - lane_width/2; 0], [0 - lane_width/2; travel], [0 - lane_width/2; travel + lane_width], [lane_width/2; travel + lane_width], [lane_width/2 + travel; travel + lane_width]];
% waypoints = [[0; 0], [0; 20 - car_length/2], [0; 20 + lane_width/2], [lane_width/2 + car_length/2; 20 + lane_width/2], [lane_width/2 + car_length/2 + 20; 20 + lane_width/2]];

% x_path = waypoints(1,:);
% y_path = waypoints(2,:);
% li_pts = 10; % number of straight line interpolation points
% x_path = [linspace(waypoints(1,1),waypoints(1,2),li_pts), linspace(waypoints(1,3), waypoints(1,4),li_pts)];
% y_path = [linspace(waypoints(2,1),waypoints(2,2),li_pts), linspace(waypoints(2,3), waypoints(2,4),li_pts)];

li_pts = 10; % number of straight line interpolation points
x_path = [linspace(waypoints(1,1),waypoints(1,2),li_pts), waypoints(1,3), linspace(waypoints(1,4), waypoints(1,5),li_pts)];
y_path = [linspace(waypoints(2,1),waypoints(2,2),li_pts), waypoints(2,3), linspace(waypoints(2,4), waypoints(2,5),li_pts)];
x_path = [x_path(1) x_path x_path(end)]; % repeat endpoints to work around midpoint spline interpolation, cuts ends short o.w.
y_path = [y_path(1) y_path y_path(end)];

hold on
% plot waypoints
plot(waypoints(1,:), waypoints(2,:), 'xb');

% midpoint spline interpolation
curve_values = spcrv([x_path; y_path]);
plot(curve_values(1,:),curve_values(2,:),'r','LineWidth',1.5);

% plot lanes
plot(lane_lower(1,:), lane_lower(2,:), 'k', 'LineWidth', 1.5);
plot(lane_upper(1,:), lane_upper(2,:), 'k', 'LineWidth', 1.5);

legend({'Waypoints' 'Fitted Spline Curve'},  'location','SE');
hold off

%% State Reference Generation

curve_x = curve_values(1,:);
curve_y = curve_values(2,:);

% Precompute heading angles along curve
curve_psi = [];
for i = 1:(length(curve_x)-1)
    dx = curve_x(i+1) - curve_x(i);
    dy = curve_y(i+1) - curve_y(i);
    curve_psi(i) = atan2(dy, dx);
end
curve_psi(end) = curve_psi(end-1); % assume the same heading for final step

% Precompute distances along curve
curve_dist = [0];
for i = 2:length(curve_x)
    dx = curve_x(i) - curve_x(i-1);
    dy = curve_y(i) - curve_y(i-1);
    curve_dist(i) = curve_dist(i-1) + sqrt(dx^2 + dy^2);
end

% Problem parameters
% Assuming constant v for now
v = 4.5; % m/s (~10 mph)
dt = 0.2;
N = 6;

% To be run at each time step before solving MPC problem

% Need a routine here to project the current state to find nearest path point
% Algorithm sketch
% 1. Begin at start point with no error
% 2. Compute N reference steps and exectue time step: x1, ..., xN
% 3. Take current state x(t) and assume the closest path point is last x1
% 4. Examine path points before and after x1. Follow direction in which distance
% between x(t) and path point decreases until it starts increasing. This assumes
% the path has decent local convexity, which is probably fine but might fail for
% e.g. parallel parking

% Initial state (x,y,v,psi)
% Assuming for now we projected onto the path
% Reset this at each time step
x0 = [0;0;v;pi/2];
% Find what point index you're at in the curve
idx = 1;
cur_dist = curve_dist(idx);
z_ref = [x0];

temp_idx = idx;
for i=1:N
    dist = cur_dist + i*v*dt;
    % find closest point in curve_dist to dist
    % or just interpolate path to have constant d(dist) between points
    check_dist = curve_dist(temp_idx);
    while(check_dist < dist)
        temp_idx = temp_idx + 1;
        check_dist = curve_dist(temp_idx);
    end
    z_ref = [z_ref, [curve_x(temp_idx); curve_y(temp_idx); v; curve_psi(temp_idx)]];
end


v = v*ones(1,length(curve_psi)); 
curve_x = curve_x(1,1:end-1);
curve_y = curve_y(1,1:end-1); 
end


%% Input Reference Generation

%% Leftover experimental code

% %% Manual parametric splines
% 
% time = 1:length(x_path);
% knots = 1:0.5:length(x_path);
% 
% % xspline = csapi(time,x_path);
% % yspline = csapi(time,y_path);
% xspline = spapi(3, time, x_path);
% yspline = spapi(3, time, y_path);
% fnplt(xspline);
% 
% % x_points = ppval(xspline, 1:0.1:length(x_path));
% % y_points = ppval(yspline, 1:0.1:length(y_path));
% 
% xmids = [];
% ymids = [];
% for i=1:length(x_path)-1
%     xmids(i) = (x_path(i) + x_path(i+1))/2;
%     ymids(i) = (y_path(i) + y_path(i+1))/2;
% end
% % plot(x_points, y_points);
% time2 = 1:length(xmids);
% xspline2 = spapi(3,time2,xmids);
% yspline2 = spapi(3,time2,ymids);
% fnplt(xspline2);
% hold on; fnplt(xspline, 'g'); hold off;
% 
% x_points = fnval(xspline2, 1:0.1:length(time2));
% y_points = fnval(yspline2, 1:0.1:length(time2));
% plot(x_points,y_points);
% 
% %%
% xpts = values(1,:);
% ypts = values(2,:);
% time = 1:length(xpts);
% 
% xspline = spapi(3,time,xpts);
% yspline = spapi(3,time,ypts);
% fnplt(xspline);
% hold on; plot(time,xpts, 'g'); hold off;
% %%
% x_points = fnval(xspline2, 1:0.1:length(time2));
% y_points = fnval(yspline2, 1:0.1:length(time2));
% plot(x_points,y_points);