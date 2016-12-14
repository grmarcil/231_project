%%  Create and export planned path
lane_length = 30;
lane_width = 5;
path = Generate_Path(lane_length,lane_width);
path_export = [path.x; path.y; path.psi; path.dist];
csvwrite('path.csv',path_export);
% now run `julia kin_bike_mpc.jl` from a terminal
% then run Julia_after.m for graphs and visualization