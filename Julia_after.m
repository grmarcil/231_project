%% Import and visualization for kin_bike_mpc.jl
% See Julia_setup.m for instructions
path = Generate_Path();
u_cl = csvread('u_cl.csv');
z_cl = csvread('z_cl.csv');

%% Plots
lane_width = 3;
lane_length = 30;
animation = Plot_Simulation(z_cl, u_cl, lane_width, lane_length, path);
%% Plot Results
figure;
plot(z_cl(1,:)); grid on
hold on;
plot(z_ref_cl(1,:),'r'); grid on
legend(' x Closed loop actual','Reference'); 

figure;
plot(z_cl(2,:)); grid on
hold on;
plot(z_ref_cl(2,:),'r'); grid on
legend(' y Closed loop actual','Reference'); 

% figure;
% plot(z_cl(3,:)); grid on
% hold on;
% plot(z_ref_cl(3,:),'r'); grid on
% legend(' v Closed loop actual','Reference'); 

figure;
plot(z_cl(4,:)); grid on
hold on;
plot(z_ref_cl(4,:),'r'); grid on
legend(' \psi Closed loop actual','Reference');

figure;
plot(u_cl(1,:)); grid on
hold on;
plot(u_ref_cl(1,:),'r'); grid on
legend(' acceleration Closed loop actual','Reference');

figure;
plot(u_cl(2,:)); grid on
hold on;
plot(u_ref_cl(2,:),'r'); grid on
legend(' \beta Closed loop actual','Reference');