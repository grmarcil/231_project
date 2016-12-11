%%  Ref tracking 

yalmip('clear')
clear all
close all
warning off

%% Model data
lf=.35;
lr=.35;
TS= 0.02;
nx = 4;      % Number of states
nu = 2;      % Number of inputs
ny = 4;      % Number of outputs

%% MPC data
C = eye(4); 
Q = eye(4);
P = Q;
R = .0001*eye(2);
Rbar = 100*eye(2);
N = 6;

%% Defining Variables

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
up = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
uref = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
xref = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
options = sdpsettings('solver','fmincon','verbose',0);
%% Setting Up Optimization Problem

constraints =[];
objective = 0;

%%
for k = 1:N
    
    objective = objective + (x{k}-xref{k})'*Q*(x{k}-xref{k}) + u{k}'*R*u{k} + (u{k}-up{k})'*Rbar*(u{k}-up{k});
    
    constr =  x{k+1}(1)== x{k}(1) + x{k}(3).*cos(x{k}(4)+u{k}(2)).*TS;
    constr = [constr; x{k+1}(2)== x{k}(2) + x{k}(3).*sin(x{k}(4)+u{k}(2)).*TS];
    constr = [constr; x{k+1}(3)==x{k}(3) + u{k}(1).*TS];
    constr = [constr; x{k+1}(4)==x{k}(4) + x{k}(3)/lr.*sin(u{k}(2)).*TS];
    
    
constru = [u{k}(2)<=.6; -u{k}(2)<=.6];
    
       
    if k~=1 && k~=N
      constru = [constru, up{k}==u{k-1}];
    
    end 
        
      constru = [constru; u{k}(1)<=1.5*TS; -u{k}(1)<=1.5*TS];
      constru = [constru; up{k}(1)<=1.5*TS; -up{k}(1)<=1.5*TS];      
%     constrx = [x{k}(1)<=20; -x{k}(1)<=20]; 
%     constrx = [constrx; x{k}(2)<=10; -x{k}(2)<=5];
%     constrx = [constrx; x{k}(3)<=10; -x{k}(3)<=10];
%     constrx = [constrx; x{k}(4)<=2*pi; -x{k}(4)<=2*pi];
      
      constrxref = norm(x{k}-xref{k},inf)<=5;
    
%     construx = [constru, constrx];
      
    construx = [constru,constrxref];
    
    constraints = [constraints, constr];
    constraints = [constraints, construx];

end

objective = objective + (x{N+1}-xref{N+1})'*P*(x{N+1}-xref{N+1});

%% Constructing reference signal

[ref_x,ref_y,ref_v,ref_psi] = Path_Generation(); 

ref = [ref_x;ref_y;ref_v;ref_psi];

%% Initializing  Control Desing

xk = ref(:,1);
xclloop(:,1) = xk;
umpc_closedloop=[];
Lsim = 100; % Length of simulation


%% Simulating MPC controller

for i = 1:Lsim
      
    % Constructing reference preview
    
    future_r = ref(:,i:N+i);    
    

    xrefk = future_r;

    
    %% Reference trajectory in closed loop
    xrefk_clloop(:,i) = xrefk(:,1); 
    
    %% Solving MPC
    
    constraints2=[];
    constraints2 = [constraints,x{1}==xk];
    
    for j=1:N+1

        constraints2 = [constraints2,xref{j}==xrefk(:,j)];
    end
    
 
    options = sdpsettings('solver','fmincon','verbose',1);
    optimize(constraints2,objective,options);
    U=double(u{1});
    
    umpc_closedloop=[umpc_closedloop,U];
    

    %% Plant model update

    xk(1) = xk(1)+xk(3).*cos(xk(4)+U(2)).*TS;
    xk(2) = xk(2)+xk(3).*sin(xk(4)+U(2)).*TS;
    xk(3) = xk(3)+U(1).*TS;
    xk(4)=  xk(4)+xk(3)/lr.*sin(U(2)).*TS;
    
    xk = [xk(1) xk(2) xk(3) xk(4)]';
    
    xclloop(:,i+1) = xk; 
    
    counter = i
    
end

%% Plot Results
figure;
Plot_Simulation(xclloop, umpc_closedloop, 3, 10)

figure;
plot(xclloop(1,:)); grid on
hold on;
plot(xrefk_clloop(1,:),'r'); grid on
legend(' x Closed loop actual','Reference');
figure;
plot(xclloop(2,:)); grid on
hold on;
plot(xrefk_clloop(2,:),'r'); grid on
legend(' y Closed loop actual','Reference');
figure;
plot(xclloop(3,:)); grid on
hold on;
plot(xrefk_clloop(3,:),'r'); grid on
legend(' v Closed loop actual','Reference');
figure;
plot(xclloop(4,:)); grid on
hold on;
plot(xrefk_clloop(4,:),'r'); grid on
legend(' \psi Closed loop actual','Reference');
%% 
% OL = Open Loop
% CL = Closed Loop
