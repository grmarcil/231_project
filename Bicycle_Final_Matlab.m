%%  Ref tracking
yalmip('clear')
clear all
close all
warning off

%% Model data
% Kinematic bicycle mode with (x,y,v,psi)
lf=1.738;
lr=1.738;
lane_width = 3;
lane_length = 30; % warning duplicate variable with Generate_Path, should parameterize
car_width = 1.85;
car_length = 4.9;
turn_radius = 5.56;

v = 4.5; % m/s (~10 mph) constant velocity for now
dt = 0.1;

nz = 4;      % Number of states
nu = 2;      % Number of inputs
ny = 4;      % Number of outputs

%% MPC data
Q = 10*[1 0 0 0;
    0 1 0 0;
    0 0 0 0;
    0 0 0 1];
P = Q;
R = eye(2);

N = 8; % MPC Horizon
Lsim = 150; % Length of simulation

%% Decision Variables
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
z = sdpvar(repmat(nz,1,N+1),repmat(1,1,N+1));
% uref = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
% xref = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
options = sdpsettings('solver','fmincon','verbose',0,'usex0',1);

%% Plan ego vehicle path
path = Generate_Path();
z0 = [path.x(1); path.y(1); v; path.psi(1)];

%% Initialize MPC Problem
z_k = z0;
z_cl(:,1) = z_k;
z_ref_cl = [];
u_ref_cl = [];
u_cl = [];
% this variable tracks distance travelled along the curve so far to speed up
% projecting actual state onto the planned path
predicted_dist = 0;

constraints = [];
objective = 0;

for k = 1:N
    % Model Dynamics
    constraints = z{k+1}(1) == z{k}(1) + z{k}(3)*cos(z{k}(4)+u{k}(2))*dt;
    constraints = [constraints; z{k+1}(2) == z{k}(2) + z{k}(3)*sin(z{k}(4)+u{k}(2))*dt];
    constraints = [constraints; z{k+1}(3) == z{k}(3) + u{k}(1)*dt];
    constraints = [constraints; z{k+1}(4) == z{k}(4) + z{k}(3)/lr*sin(u{k}(2))*dt];
    
    constru = [u{k}(2)<=0.6; -u{k}(2)<=0.6];
    constru = [constru; u{k}(1)<=1.5*dt; -u{k}(1)<=1.5*dt];
    constrx = [z{k}(1)<=30; -z{k}(1)<=30];
    constrx = [constrx; z{k}(2)<=30; -z{k}(2)<=30];
    constrx = [constrx; z{k}(3)<=10; -z{k}(3)<=10];
    constrx = [constrx; z{k}(4)<=2*pi; -z{k}(4)<=2*pi];
    
    construx = [constru, constrx];
    
    constraints = [constraints, construx];
    
end

%% Run MPC simulation

for i = 1:Lsim
    
    [z_ref, u_ref, path_dist] = Generate_Ref(z_k, path, predicted_dist, N, dt, lr);
    
 %% Changes made from here on to stop and turn 
    
    if (all(diff(z_ref(4,:))==0)) % If straight line ref in a horizon, keep solving MPC  
    
    for k = 1:N
        objective2 = objective + (z{k}-z_ref(:,k))'*Q*(z{k}-z_ref(:,k)) + (u{k}-u_ref(:,k))'*R*(u{k}-u_ref(:,k));
        % objective2 = objective + (z{k}-z_ref(:,k))'*Q*(z{k}-z_ref(:,k));
    end
    objective2 = objective2 + (z{N+1}-z_ref(:,N+1))'*P*(z{N+1}-z_ref(:,N+1));
    
    constraints2 = [constraints,z{1}==z_cl(:,i)];
    
    % Initial guesses
    for k = 1:N
%         dz = 0.1*rand(4,1) - 0.05;
%         du = 0.1*rand(2,1) - 0.05;
        assign(z{k},z_ref(:,k));
        assign(u{k},u_ref(:,k));
    end
%     dz = 0.1*rand(4,1) - 0.05;
    assign(z{N+1},z_ref(:,N+1));
    
    diag = solvesdp(constraints2,objective2,options);
    diagnostics = diag.problem;
    
    % Process output
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    u_k = double(u{1});
    u_cl = [u_cl,u_k];
    
    %% Simulate plant update
    % Temp calculation vars
    z_k1 = z_k(1) + z_k(3)*cos(z_k(4)+u_k(2))*dt;
    z_k2 = z_k(2) + z_k(3)*sin(z_k(4)+u_k(2))*dt;
    z_k3 = z_k(3) + u_k(1)*dt;
    z_k4 = z_k(4) + z_k(3)/lr*sin(u_k(2))*dt;
    
    predicted_dist = path_dist + z_k(3)*dt;
    z_k = [z_k1; z_k2; z_k3; z_k4];
    
    % Update CL trajectories
    z_cl(:,i+1) = z_k;
    z_ref_cl(:,i) = z_ref(:,1);
    u_ref_cl(:,i) = u_ref(:,1);
    u_cl(:,i) = u_k;
    endstri = i;
    
    counter = i
    
    end
    
end

u_cl = u_cl(:,endstri); 
z_k = z_cl(:,endstri);

%% After a horizon reference denotes psi change, i.e. impending curve, stop MPC and apply braking 
for i=endstri:Lsim
      
    
      
    if (z_k(3)>0)   % Reduce positive velocity to 0 by braking; no MPC 
    
    u_k = [-z_k(3)/(3*dt),0]';  % Acceleration constraint violated. 
    u_cl = [u_cl, u_k];
    
    %% Simulate plant update
    % Temp calculation vars
    z_k1 = z_k(1) + z_k(3)*cos(z_k(4)+u_k(2))*dt;
    z_k2 = z_k(2) + z_k(3)*sin(z_k(4)+u_k(2))*dt;
    z_k3 = z_k(3) + u_k(1)*dt;
    z_k4 = z_k(4) + z_k(3)/lr*sin(u_k(2))*dt;
    
    predicted_dist = path_dist + z_k(3)*dt;
    z_k = [z_k1; z_k2; z_k3; z_k4];
    
    % Update CL trajectories
    z_cl(:,i+1) = z_k;
    z_ref_cl(:,i) = z_ref(:,1);
    u_ref_cl(:,i) = u_ref(:,1);
    u_cl(:,i) = u_k;
    
    end

     counter = i
 
end

%% Plots
animation = Plot_Simulation(z_cl, u_cl, lane_width, lane_length, path);


% % % Plot Results
% % figure;
% % plot(z_cl(1,:)); grid on
% % hold on;
% % plot(z_ref_cl(1,:),'r'); grid on
% % legend(' x Closed loop actual','Reference');
% % 
% % figure;
% % plot(z_cl(2,:)); grid on
% % hold on;
% % plot(z_ref_cl(2,:),'r'); grid on
% % legend(' y Closed loop actual','Reference');
% % 
% % figure;
% % plot(z_cl(3,:)); grid on
% % hold on;
% % plot(z_ref_cl(3,:),'r'); grid on
% % legend(' v Closed loop actual','Reference');
% % 
% % figure;
% % plot(z_cl(4,:)); grid on
% % hold on;
% % plot(z_ref_cl(4,:),'r'); grid on
% % legend(' \psi Closed loop actual','Reference');
% % 
% % figure;
% % plot(u_cl(1,:)); grid on
% % hold on;
% % plot(u_ref_cl(1,:),'r'); grid on
% % legend(' acceleration Closed loop actual','Reference');
% % 
% % figure;
% % plot(u_cl(2,:)); grid on
% % hold on;
% % plot(u_ref_cl(2,:),'r'); grid on
% % legend(' \beta Closed loop actual','Reference');