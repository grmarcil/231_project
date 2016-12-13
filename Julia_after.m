%% Import and visualization for kin_bike_mpc.jl
% See Julia_setup.m for instructions
path = Generate_Path();
filename = 'mpc_sim.h5';
u_cl = h5read(filename,'/u_cl');
z_cl = h5read(filename,'/z_cl');
u_ref_cl = h5read(filename,'/u_ref_cl');
z_ref_cl = h5read(filename,'/z_ref_cl');

%% Update parameters to fit julia data as needed
refs=149;
N=5;
lane_width = 3;
lane_length = 30;
%% Simulation
animation = Plot_Simulation(z_cl, u_cl, lane_width, lane_length, path);
%% Plot Results
figure
grid on
subplot(4,1,1)
hold on
plot(z_cl(1,:))
for i=1:refs
    plot(i:i+N,z_ref_cl(1,:,i),'r')
end
legend(' x Closed loop actual','N-step Reference'); 

subplot(4,1,2)
hold on
plot(z_cl(2,:))
for i=1:refs
    plot(i:i+N,z_ref_cl(2,:,i),'r')
end
legend(' y Closed loop actual','N-step Reference'); 

subplot(4,1,3)
hold on
plot(z_cl(3,:))
for i=1:refs
    plot(i:i+N,z_ref_cl(3,:,i),'r')
end
legend(' \psi Closed loop actual','N-step Reference'); 

subplot(4,1,4)
hold on
plot(z_cl(4,:))
for i=1:refs
    plot(i:i+N,z_ref_cl(4,:,i),'r')
end
legend(' v Closed loop actual','N-step Reference');
datetime_str = regexprep(regexprep(datestr(datetime),'-',''),'[\s:]','-');
fig1 = ['figures/', datetime_str, '_z.png'];
export_fig(fig1)
%%
figure;
subplot(2,1,1)
hold on
plot(u_cl(1,:))
for i=1:refs
    plot(i:i+N-1,u_ref_cl(1,:,i),'r')
end
legend(' acceleration Closed loop actual','N-step Reference');

subplot(2,1,2)
hold on
plot(u_cl(2,:))
for i=1:refs
    plot(i:i+N-1,u_ref_cl(2,:,i),'r')
end
legend(' \beta Closed loop actual','N-step Reference');
hold off
fig1 = ['figures/', datetime_str, '_u.png'];
export_fig(fig1)