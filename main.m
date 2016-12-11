%% ME C231A Project: Urban Driving
% Main execution file
% Group

%% Dimensions (all sizes in meters)
lane_width = 3;
lane_length = 10;

car_width = 1.85;
car_length = 4.9;
turn_radius = 5.56;

%% Example of Generate_Path
% points for a right hand turn

% shorthand vars
lw = lane_width;
ll = lane_length;
waypoints = [[lw/2; -ll-lw], [lw/2;  -lw-car_length/2], [lw/2; -lw/2], [lw + car_length/2; -lw/2], [lw+ll;  -lw/2]];

path = Generate_Path();

hold on
% plot waypoints
plot(waypoints(1,:), waypoints(2,:), 'xb');
% plot fitted path
plot(path.x,path.y,'r','LineWidth',1.5);

legend({'Waypoints' 'Fitted Spline Curve'},  'location','SE');
hold off

%% Example of Generate_Ref

% Initial state (x,y,v,psi)
z0 = [lw/2;-ll-lw;v;pi/2];
% Find distance along path and next N steps of z_ref
[z_ref, u_ref, dist] = Generate_Ref(z0, curve, 0, N, dt,1.738);
% Plot z_ref
hold on; plot(z_ref(1,:), z_ref(2,:), 'xg'); hold off;
%% Input Reference Generation