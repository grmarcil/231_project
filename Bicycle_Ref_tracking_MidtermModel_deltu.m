%%  Ref tracking 

yalmip('clear')
clear all
close all
warning off

%% Model data
TS=0.2;
nx = 3;      % Number of states
nu = 2;      % Number of inputs
ny = 3;      % Number of outputs

%% MPC data
C = eye(3); 
Q = 10*eye(3); 
P = Q;
R = 5*eye(2);
Rbar = 5*eye(2);
N = 6;
L = 0.7;
delmax = 25*pi/180;

%% Defining Variables

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
up = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
uref = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
xref = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
options = sdpsettings('solver','fmincon','verbose',1);
%% Setting Up Optimization Problem

constraints =[];
objective = 0;

for k = 1:N
    
    objective = objective + (x{k}-xref{k})'*Q*(x{k}-xref{k}) + u{k}'*R*u{k} + (u{k}-up{k})'*Rbar*(u{k}-up{k});
    
    
    constr =  x{k+1}(1)== x{k}(1) - u{k}(1).*sin(x{k}(3)) + u{k}(1).*sin(x{k}(3)+u{k}(2));
    constr = [constr; x{k+1}(2)== x{k}(2) + u{k}(1).*cos(x{k}(3)) - u{k}(1).*cos(x{k}(3)+u{k}(2))];
    constr = [constr; x{k+1}(3)==x{k}(3) + u{k}(2)];

    
    
      constru = [u{k}(2)<=pi; -u{k}(2)<=pi];

%     constru = [constru; u{k}(1)<=inf; -u{k}(1)<=L/tan(delmax)];

      constrxref = [norm(x{k}-xref{k},inf)<=0.4];
    
%     constrx = [x{k}(1)<=20; -x{k}(1)<=20]; 
%     constrx = [constrx; x{k}(2)<=10; -x{k}(2)<=5];
%     constrx = [constrx; x{k}(3)<=10; -x{k}(3)<=10];
%     
     construx = [constru];
     

if k~=1 && k~=N
    constru=[constru, up{k}==u{k-1}];
end
    

   constraints = [constraints, constr];
   constraints = [constraints, construx];

end

objective = objective + (x{N+1}-xref{N+1})'*P*(x{N+1}-xref{N+1});


%% Constructing reference signal

[ref_x,ref_y,ref_v,ref_psi] = Path_Generation(); 
ref = [ref_x;ref_y;ref_psi];


%% Initializing  Control Desing

xk = ref(:,1);
xclloop(:,1) = xk;
umpc_closedloop=[];
Lsim = 100;           % Length of simulation

%% Simulating MPC controller


for i = 1:Lsim
      
    % Constructing reference preview
    
    future_r = ref(:,i:N+i);    
    
    % Computing x_ref based on future reference

    xrefk=future_r;
    
    %% Reference trajectory in closed loop
    xrefk_clloop(:,i) = xrefk(:,1); 
    
    %% Solving MPC
    
    constraints2=[];
    constraints2 = [constraints,x{1}==xk];
    for j=1:N+1

        constraints2 = [constraints2,xref{j}==xrefk(:,j)];
    end
    
   
    diag=solvesdp(constraints2,objective,options);
    diagnostics=diag.problem; 
    
    % Process output
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    U=double(u{1});
    umpc_closedloop=[umpc_closedloop,U];
    

    %% Plant model update
   
    xk(1) = xk(1)-U(1).*sin(xk(3))+U(1).*sin(xk(3)+U(2));
    xk(2) = xk(2)+U(1).*cos(xk(3))-U(1).*cos(xk(3)+U(2));
    xk(3) = xk(3)+U(2);
  
    
    xk = [xk(1) xk(2) xk(3)]';
    
    xclloop(:,i+1) = xk; 
   
    counter = i
    
end


%% Plot Results

Plot_Simulation(xclloop, umpc_closedloop, 3, 10)
