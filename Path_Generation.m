%% ME C231A Project: Urban Driving
% Main execution file
% Group

function [curve_x, curve_y, v , curve_psi ] = Path_Generation()
%% Dimensions (all sizes in meters)
lane_width = 3;
lane_length = 10;

car_width = .7;
car_length = 1.5;

turn_radius = 5.56;

%% Path generation
% points for a right hand turn
travel = 5;
% shorthand vars
lw = lane_width;
ll = lane_length;
waypoints = [[lw/2; -ll-lw], [lw/2;  -lw-car_length/2], [lw/2; -lw/2], [lw + car_length/2; -lw/2], [lw+ll;  -lw/2]];

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

legend({'Waypoints' 'Fitted Spline Curve'},  'location','SE');
hold off

%% State Reference Generation

curve.x = curve_values(1,:);
curve.y = curve_values(2,:);

% Precompute heading angles and distances along curve
curve.psi = [0];
curve.dist = [0];
for i = 2:length(curve.x)
    dx = curve.x(i) - curve.x(i-1);
    dy = curve.y(i) - curve.y(i-1);
    curve.psi(i) = atan2(dy, dx);
    curve.dist(i) = curve.dist(i-1) + sqrt(dx^2 + dy^2);
end
curve.psi(1) = curve.psi(2); % assume the same angle for first and second steps

% Enforce even spacing of all points on reference curve
new_dist = linspace(curve.dist(1), curve.dist(end), length(curve.dist));
new_x = interp1(curve.dist, curve.x, new_dist);
new_y = interp1(curve.dist, curve.y, new_dist);
new_psi = interp1(curve.dist, curve.psi, new_dist);
curve = struct('dist', new_dist, 'x', new_x, 'y', new_y, 'psi', new_psi);
% Add interpolation functions to curve struct
curve.interp_x = @(dist) interp1(curve.dist, curve.x, dist);
curve.interp_y = @(dist) interp1(curve.dist, curve.y, dist);
curve.interp_psi = @(dist) interp1(curve.dist, curve.psi, dist);

% Problem parameters
% Assuming constant v for now
v = 4.5; % m/s (~10 mph)
dt = 0.02;
N = 6;

curve_psi = curve.psi;
v = v*ones(1,length(curve_psi)); 
curve_x = curve.x;
curve_y = curve.y;
end


