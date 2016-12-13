%% Import and visualization for kin_bike_mpc.jl
% See Julia_setup.m for instructions
%% Update parameters to fit julia data as needed
close all
steps=149; % How many OL steps to plot, ie (Lsim-1)
N=5;
lane_width = 3;
lane_length = 30;

path = Generate_Path(lane_length);
filename = 'mpc_sim.h5';
u_cl = h5read(filename,'/u_cl');
z_cl = h5read(filename,'/z_cl');
u_ol = h5read(filename,'/u_ol');
z_ol = h5read(filename,'/z_ol');
u_ref_ol = h5read(filename,'/u_ref_ol');
z_ref_ol = h5read(filename,'/z_ref_ol');

%% Simulation
animation = Plot_Simulation(z_cl, u_cl, lane_width, lane_length, path);
%% Plot Results
z_fig = figure;
grid on
subplot(4,1,1)
hold on
plot(z_cl(1,:))
for i=1:steps
    plot(i:i+N,z_ol(1,:,i),'b.--')
    plot(i:i+N,z_ref_ol(1,:,i),'r')
end
legend('x closed loop','x open loop','Reference','Location','Best');

subplot(4,1,2)
hold on
plot(z_cl(2,:))
for i=1:steps
    plot(i:i+N,z_ol(2,:,i),'b.--')
    plot(i:i+N,z_ref_ol(2,:,i),'r')
end
legend('y closed loop','y open loop','Reference','Location','Best');

subplot(4,1,3)
hold on
plot(z_cl(3,:))
for i=1:steps
    plot(i:i+N,z_ol(3,:,i),'b.--')
    plot(i:i+N,z_ref_ol(3,:,i),'r')
end
legend('\psi closed loop','\psi open loop','Reference','Location','Best');

subplot(4,1,4)
hold on
plot(z_cl(4,:))
for i=1:steps
    plot(i:i+N,z_ol(4,:,i),'b.--')
    plot(i:i+N,z_ref_ol(4,:,i),'r')
end
legend('v closed loop','v open loop','Reference','Location','Best');

%%
u_fig = figure;
subplot(2,1,1)
hold on
plot(u_cl(1,:))
for i=1:steps
    plot(i:i+N-1,u_ol(1,:,i),'b.--')
    plot(i:i+N-1,u_ref_ol(1,:,i),'r')
end
legend('\beta closed loop','\beta open loop','Reference','Location','Best');

subplot(2,1,2)
hold on
plot(u_cl(2,:))
for i=1:steps
    plot(i:i+N-1,u_ol(2,:,i),'b.--')
    plot(i:i+N-1,u_ref_ol(2,:,i),'r')
end
legend('acceleration closed loop','acceleration open loop','Reference','Location','Best');

hold off

%% Export Figures
datetime_str = regexprep(regexprep(datestr(datetime),'-',''),'[\s:]','-');
z_fig_name = ['figures/', datetime_str, '_z.png'];
export_fig(z_fig, z_fig_name,'-m3')
u_fig_name = ['figures/', datetime_str, '_u.png'];
export_fig(u_fig, u_fig_name,'-m3')
vid_name = ['figures/', datetime_str, '.avi'];
vid = VideoWriter(vid_name);
open(vid);
writeVideo(vid,animation);
close(vid);