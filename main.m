%% ME C231A Project: Urban Driving
% Main execution file
% Group

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

%fnplt(cscvn([x_path; y_path]), 'g', 1.5);

% midpoint spline interpolation
values = spcrv([x_path; y_path]);
plot(values(1,:),values(2,:),'r','LineWidth',1.5);

% plot lanes
plot(lane_lower(1,:), lane_lower(2,:), 'k', 'LineWidth', 1.5);
plot(lane_upper(1,:), lane_upper(2,:), 'k', 'LineWidth', 1.5);

legend({'Waypoints' 'Fitted Spline Curve'},  'location','SE');
hold off

%% Manual parametric splines

time = 1:length(x_path);
knots = 1:0.5:length(x_path);

% xspline = csapi(time,x_path);
% yspline = csapi(time,y_path);
xspline = spapi(3, time, x_path);
yspline = spapi(3, time, y_path);
fnplt(xspline);

% x_points = ppval(xspline, 1:0.1:length(x_path));
% y_points = ppval(yspline, 1:0.1:length(y_path));

xmids = [];
ymids = [];
for i=1:length(x_path)-1
    xmids(i) = (x_path(i) + x_path(i+1))/2;
    ymids(i) = (y_path(i) + y_path(i+1))/2;
end
% plot(x_points, y_points);
time2 = 1:length(xmids);
xspline2 = spapi(3,time2,xmids);
yspline2 = spapi(3,time2,ymids);
fnplt(xspline2);
hold on; fnplt(xspline, 'g');