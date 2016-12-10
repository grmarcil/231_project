%%  Ref tracking 

yalmip('clear')
clear all
close all
warning off

%% Model data
lf=1.738;
lr=1.738;
TS=0.2;
nx = 4;      % Number of states
nu = 2;      % Number of inputs
ny = 4;      % Number of outputs

%% MPC data
C = eye(4); 
Q = 10*eye(4); 
P = Q;
R = 5*eye(2);
N = 6;

%% Defining Variables

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
uref = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
xref = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
options = sdpsettings('solver','fmincon','verbose',0);
%% Setting Up Optimization Problem

constraints =[];
objective = 0;

for k = 1:N
    
    objective = objective + (x{k}-xref{k})'*Q*(x{k}-xref{k}) + (u{k}-uref{k})'*R*(u{k}-uref{k});
    
    
    constr =  x{k+1}(1)== x{k}(1) + x{k}(3).*cos(x{k}(4)+u{k}(2)).*TS;
    constr = [constr; x{k+1}(2)== x{k}(2) + x{k}(3).*sin(x{k}(4)+u{k}(2)).*TS];
    constr = [constr; x{k+1}(3)==x{k}(3) + u{k}(1).*TS];
    constr = [constr; x{k+1}(4)==x{k}(4) + x{k}(3)/lr.*sin(u{k}(2)).*TS];

    
    
%     constru = [u{k}(2)<=0.6; -u{k}(2)<=0.6];
%     constru = [constru; u{k}(1)<=1.5*TS; -u{k}(1)<=1.5*TS];
%     constrx = [x{k}(1)<=20; -x{k}(1)<=20]; 
%     constrx = [constrx; x{k}(2)<=10; -x{k}(2)<=5];
%     constrx = [constrx; x{k}(3)<=10; -x{k}(3)<=10];
%     constrx = [constrx; x{k}(4)<=2*pi; -x{k}(4)<=2*pi];
%     
%     construx = [constru, constrx];
%     
%     
    constraints = [constraints, constr];
%   constraints = [constraints, construx];

end

objective = objective + (x{N+1}-xref{N+1})'*P*(x{N+1}-xref{N+1});


%% Constructing reference signal

% for i = 1:Lsim+N+1
%     if i<40
%         ref=[ref,zeros(4,1)];
%     else
%         ref=[ref,[10; 0 ; 1 ; 0]];
%     end
% end

[ref_x,ref_y,ref_v,ref_psi] = Path_Generation(); 
ref = [ref_x;ref_y;ref_v;ref_psi];


%% Initializing  Control Desing

xk = ref(:,1);
xclloop(:,1) = xk;
umpc_closedloop=[];
Lsim = 150; % Length of simulation



%% Simulating MPC controller


for i = 1:Lsim
      
    % Constructing reference preview
    
    future_r = ref(:,i:N+i);    
    
    % Computing x_ref and u_ref based on future reference
    xrefk=[];
    urefk=[];

for l=1:length(future_r)
    
    ur = sdpvar(2,N);
    assign(ur,zeros(2,N));
  

        
    term1 =  ur(2,(1:end)).*TS-(future_r(1,(2:end))-future_r(1,(1:end-1))-future_r(3,(1:end-1)).*cos(future_r(4,(1:end-1))));
    term2 = future_r(2,(2:end))-future_r(2,(1:end-1))-(future_r(3,(1:end-1)).*sin(future_r(4,(1:end-1))+ur(2,(1:end))).*TS);
    term3 = future_r(3,(2:end))-future_r(3,(1:end-1))-(ur(1,(1:end)).*TS);
    term4 = future_r(4,(2:end))-future_r(4,(1:end-1))-(future_r(3,(1:end-1))/lr.*sin(ur(2,(1:end))).*TS);
  
      
%     cost = norm(ur)^2;

      cost = norm(ur)^2 + 50*(norm(term1)^2+norm(term2)^2+norm(term3)^2+norm(term4)^2);
    

%     constr1 =  ur(2,(1:end)).*TS == (future_r(1,(2:end))-future_r(1,(1:end-1))-future_r(3,(1:end-1)).*cos(future_r(4,(1:end-1))));
%     constr1 = [constr1; future_r(2,(2:end))-future_r(2,(1:end-1))== future_r(3,(1:end-1)).*sin(future_r(4,(1:end-1))+ur(2,(1:end))).*TS];
%     constr1 = [constr1; future_r(3,(2:end))-future_r(3,(1:end-1))== ur(1,(1:end)).*TS];
%     constr1 = [constr1; future_r(4,(2:end))-future_r(4,(1:end-1))== future_r(3,(1:end-1))/lr.*sin(ur(2,(1:end))).*TS];


%     constr3 = [ur(2,:)<=0.6; -ur(2,:)<=0.6];
%     constr3 = [constr3; ur(1,:)<=1.5*TS; -ur(1,:)<=1.5*TS];
% 
% %     constr13 = [constr1,constr3];
%  
%     constr13 = constr3;
%     
    options = sdpsettings('solver','ipopt','verbose',1,'usex',0);
    
    optimize([],cost,options);
    
    
    urr=double(ur);
    
    xrefk=[xrefk,future_r(:,l)];
    urefk=[urefk,urr(:,end)];
    
end

%  end
    
    %% Reference trajectory in closed loop
    xrefk_clloop(:,i) = xrefk(:,1); 
    
    %% Solving MPC
    
    constraints2=[];
    constraints2 = [constraints,x{1}==xk];
    for j=1:N+1

        constraints2 = [constraints2,xref{j}==xrefk(:,j)];
    end
    
    for j=1:N
       constraints2 = [constraints2,uref{j}==urefk(:,j)];
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
