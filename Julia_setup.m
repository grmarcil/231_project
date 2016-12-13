%%  Create and export planned path
lane_length = 30;
path = Generate_Path(lane_length);
path_export = [path.x; path.y; path.psi; path.dist];
csvwrite('path.csv',path_export);
% now run `julia kin_bike_mpc.jl` from a terminal
% then run Julia_after.m for graphs and visualization