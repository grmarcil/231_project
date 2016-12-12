%% Plot Simulation
% Greg Marcil, 10/Dec/2016
% Creates animation of a car passing through a two-way intersection
% ------------------------------------------------------------------------------
% Plot_Simulation(z_cl, z_ref, u_cl, lane_width, lane_length, path)
% ------------------------------------------------------------------------------
% z_cl: closed loop trajectory of the ego vehicle (array of 4x1 state vectors
% (x,y,psi,v) ([m],[m],[rad],[m/s]
% waypoints: target points used to generate ref trajectory
% z_ref: state reference trajectory
% u_cl: closed loop input trajectory of the ego vehicle (array of 2x1 input
% vectors (accel, steering angle) ([m/s^2], [rad])
% lane_width: width of one lane ([m])
% lane_length: length of roadway before intersection ([m])
% path: path struct
% ------------------------------------------------------------------------------
% animation: matlab animation, play with movie(animation)
% ------------------------------------------------------------------------------

function animation = Plot_Simulation(z_cl, u_cl, lane_width, lane_length, path)
% The environment is a four way intersection. The origin is the middle of the
% intersection

    %% Define car
    % Full refactor should define auto in main.m and pass it as a parameter to Plot_Simulation
    auto.w = 1.85;                % car width [m]
    auto.l = 4.9;                % car length [m]
    auto.db = 0.71;               % rear axis position, from back [m]
    auto.df = 0.71;               % front axis position, from front [m]
    auto.tyr = 0.8;              % tyre diameter [m]
    auto.dmax = 25*pi/180;       % maximum front wheel steering angle [rad]
    auto.drat = 14.5;            % ratio between steering wheel angle and front wheel angle
    auto.d = auto.l - auto.df - auto.db;  % axis distance [m]

    %% Define roadmarkings
    % shorthand variables
    lw = lane_width;
    ll = lane_length;

    % horizontal lane edges
    left_low = [[-lw-ll; -lw], [-lw; -lw]];
    left_high = [[-lw-ll; lw], [-lw; lw]];
    right_low = [[lw+ll; -lw], [lw; -lw]];
    right_high = [[lw+ll; lw], [lw; lw]];
    % veritcal lane edges
    bottom_left = [[-lw; -lw-ll], [-lw; -lw]];
    bottom_right = [[lw; -lw-ll], [lw; -lw]];
    top_left = [[-lw; lw+ll], [-lw; lw]];
    top_right = [[lw; lw+ll], [lw; lw]];

    % centerlines
    centerline_n = [[0; lw+ll], [0; lw]];
    centerline_e = [[lw+ll; 0], [lw; 0]];
    centerline_s = [[0; -lw-ll], [0; -lw]];
    centerline_w = [[-lw-ll; 0], [-lw; 0]];

    % stop stripes
    stop_n = [[-lw; lw], [0; lw]];
    stop_s = [[0; -lw], [lw; -lw]];

    % collect line segments for plotting
    edges = {left_low, left_high, right_low, right_high, bottom_left, bottom_right, top_left, top_right};
    centerlines = {centerline_n, centerline_e, centerline_s, centerline_w};
    stops = {stop_n, stop_s};

    %% Plot scene
    fig = figure; hold on;
    set(gca,'Color',[0.3 0.3 0.3]);
    % Planned path
    plot(path.x, path.y,'r','LineWidth',1.5);
    % Road markings
    for i=1:length(edges)
        plot(edges{i}(1,:), edges{i}(2,:), 'w', 'LineWidth', 2);
    end
    for i=1:length(centerlines)
        plot(centerlines{i}(1,:), centerlines{i}(2,:), 'y--', 'LineWidth', 2);
    end
    for i=1:length(stops)
        plot(stops{i}(1,:), stops{i}(2,:), 'w', 'LineWidth', 5);
    end
    hold off;
    axis([-lw-ll lw+ll -lw-ll lw+ll]);
    axis square;
    %% Plot car
    u_cl(:,end+1) = u_cl(:,end); % set steering angle for last frame

    animation(length(z_cl)) = struct('cdata',[],'colormap',[]);
    for i=1:length(z_cl)
        p = Plot_Car(z_cl(:,i), u_cl(:,i), auto, fig, [0.9 0.9 0.9]);
        animation(i) = getframe;
        delete(p);
    end
end
