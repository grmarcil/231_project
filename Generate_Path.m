%% Generate Path
%
% Generate the ego vehicle's desired navigation path
% ------------------------------------------------------------------------------
% Generate_Path()
% ------------------------------------------------------------------------------
% lane_length: length of all four roads approaching the intersection [m]
% lane_width: width of a single lane [m]
% ------------------------------------------------------------------------------
% path: a struct containing dist, x, y, and psi trajectories for the ego vehicle
% to follow. The struct also includes interpolation functions for x, y, and psi
% according to distance travelled along the path
% ------------------------------------------------------------------------------

function path = Generate_Path(lane_length, lane_width)
    car_length = 4.9;
    % shorthand vars
    lw = lane_width;
    ll = lane_length;
    % car begins at bottom of vertical lane, make a right turn through
    % intersection, then follows to end of horizontal lane
    % Right Hand Turn
    %waypoints = [[lw/2; -ll-lw], [lw/2;  -lw-car_length/2], [lw/2; -lw/2], [lw + car_length/2; -lw/2], [lw+ll;  -lw/2]];
    % Left Hand Turn
    waypoints = [[lw/2; -ll-lw], [lw/2;  -lw-car_length/2], [lw/2; lw/2], [-lw - car_length/2; lw/2], [-lw-ll;  lw/2]];

    % Interpolate straight line segments before curve fitting
    interp_pts = 10; % number of straight line interpolation points
    x_path = [linspace(waypoints(1,1),waypoints(1,2),interp_pts), waypoints(1,3), linspace(waypoints(1,4), waypoints(1,5),interp_pts)];
    y_path = [linspace(waypoints(2,1),waypoints(2,2),interp_pts), waypoints(2,3), linspace(waypoints(2,4), waypoints(2,5),interp_pts)];
    x_path = [x_path(1) x_path x_path(end)]; % repeat endpoints to work around midpoint spline interpolation, cuts ends short o.w.
    y_path = [y_path(1) y_path y_path(end)];

    % Perform midpoint spline interpolation to get a smoothed path from discrete
    % waypoints
    path_values = spcrv([x_path; y_path]);

    path.x = path_values(1,:);
    path.y = path_values(2,:);

    % Compute heading angles and distances along curve
    path.psi = [0];
    path.dist = [0];
    for i = 2:length(path.x)
        dx = path.x(i) - path.x(i-1);
        dy = path.y(i) - path.y(i-1);
        path.psi(i) = atan2(dy, dx);
        path.dist(i) = path.dist(i-1) + sqrt(dx^2 + dy^2);
    end
    path.psi(1) = path.psi(2); % assume the same angle for first and second steps

    % Enforce even spacing of all points on the path
    new_dist = linspace(path.dist(1), path.dist(end), length(path.dist));
    new_x = interp1(path.dist, path.x, new_dist);
    new_y = interp1(path.dist, path.y, new_dist);
    new_psi = interp1(path.dist, path.psi, new_dist);
    path = struct('dist', new_dist, 'x', new_x, 'y', new_y, 'psi', new_psi);
    
    % Add interpolation functions to path struct
    path.interp_x = @(dist) interp1(path.dist, path.x, dist);
    path.interp_y = @(dist) interp1(path.dist, path.y, dist);
    path.interp_psi = @(dist) interp1(path.dist, path.psi, dist);
end