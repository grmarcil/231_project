function car_plot(u,z0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the motion of a car according to the model:
% {x}_{k+1} &=& x_k-R_k*sin(\theta_k)+R_k*sin(\theta_k+\beta_k) 
%{y}_{k+1} &=& y_k+R_k*cos(\theta_k)-R_k*cos(\theta_k+\beta_k) 
%{\theta}_{k+1} &=& \theta_k+\beta_k 
%---------------------------------------------------
% with inputs u=[R_k,\beta_k] and state z=[z,y,\theta]
% u is  2xN matrix where u(:,k) are the two inputs at time k
% z0 is a 3x1 vector of initial conditions.
%--------------------------------------------
% In the plot, red is the front of the vehicle

% Sampling parameter
rep=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation of a simple kinematic vehicle
t=1;
xsim=[];
usim=[];
xsim(:,1)=z0;
N=size(u,2);
for k = 1:N        
    for j = 1:rep        
         t=t+1;
         xsim(1,t) =xsim(1,t-1) -u(1,k)*sin(xsim(3,t-1))+u(1,k)*sin(xsim(3,t-1)+1/rep*u(2,k));
         xsim(2,t) =xsim(2,t-1) +u(1,k)*cos(xsim(3,t-1))-u(1,k)*cos(xsim(3,t-1)+1/rep*u(2,k));
         xsim(3,t) = xsim(3,t-1) + 1/rep*u(2,k);
         usim(:,t-1)=[u(1,k);1/rep*u(2,k)];
    end
end

usim(:,t)=usim(:,t-1);
X=xsim';
U=usim';

% 1.4) Car Model (for Dynamic Programming)

auto.w = .7;                % car width [m]
auto.l = 1.50;                % car length [m]
auto.db = .4;               % rear axis position, from back [m]
auto.df = .4;               % front axis position, from front [m]
auto.tyr = 0.4;              % tyre diameter [m]
auto.dmax = 25*pi/180;       % maximum front wheel steering angle [rad]
auto.drat = 14.5;            % ratio between steering wheel angle and front wheel angle
auto.d = auto.l - auto.df - auto.db;  % axis distance [m]

%Compute axis limits here
xmin=min(xsim(1,:));
xmax=max(xsim(1,:));
ymin=min(xsim(2,:));
ymax=max(xsim(2,:));
L=max(xmax-xmin,ymax-ymin);
limits=[(xmin+xmax)/2-L/2-2,(xmin+xmax)/2+L/2+2,(ymin+ymax)/2-L/2-2,(ymin+ymax)/2+L/2+2];


% Trajectory
plottraj.linestyle = '-';
plottraj.linewidth = 1;
plottraj.color = [0 0 1]; % RGB value
plottraj.marker = 'none';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Vehicle trajectory (oversampled, to show the path)
for i = 1:N*rep+1
    p = plotcar(X(i,1),X(i,2),X(i,3),U(i,2),auto,[],[0.3 0.3 0.3]);
    pause(0.002)
    if i<=N*rep & i>1 
        delete(p)
    end
    if i<N*rep
        plot([X(i,1),X(i+1,1)],[X(i,2),X(i+1,2)],plottraj);
    end
end
% for i = 1:rep:N*rep+1
%  plot(X(i,1),X(i,2),'go')
%  [u,v] = pol2cart(X(i,3),1);
%  quiver(X(i,1),X(i,2),u,v,'g','linewidth',2,'maxheadsize',2);
% end

%%%%%%
% Plot model states

% Georg Schildbach, 15/Dec/2013 --- Automobile
% Plots the geometric data of the car
% --------------------------------------------------------------------------------------------------
% plotcar(x,y,phi,delta,auto,fig,rgb)
% --------------------------------------------------------------------------------------------------
% x,y,phi: coordinates of the car (in [m], [m], [rad])
% delta: steering angle [rad]
% auto: structure with geometric data of the car
% fig: figure handle
% rgb: red-green-blue value (1x3 vector)
% --------------------------------------------------------------------------------------------------
% P: plot handle (5x1 vector)
% --------------------------------------------------------------------------------------------------

function P = plotcar(x,y,phi,delta,auto,~,rgb)

% 1) Compute Car Geometry --------------------------------------------------------------------------

ageom = autogeometry(x,y,phi,delta,auto);
 
% 2) Plot Options ----------------------------------------------------------------------------------

% 2.1) Car
car_op.linestyle = '-';
car_op.color = rgb; % RGB value
car_op.marker = 'none';
car_op.linewidth = 2;

% 2.2) Tyres
tyre_op.linestyle = '-';
tyre_op.color = rgb; % RGB value
tyre_op.marker = 'none';
tyre_op.linewidth = 4;

tyre_opf.linestyle = '-';
tyre_opf.color = 'red'; % RGB value
tyre_opf.marker = 'none';
tyre_opf.linewidth = 4;

% 2.3) Obstacles
obstacle_op.linestyle = 'none';
obstacle_op.marker = '.';
obstacle_op.markersize = 7;
obstacle_op.markeredgecolor = [0 0 0]; 
obstacle_op.markerfacecolor = [0 0 0];

% 3) Plot ------------------------------------------------------------------------------------------

figure(1)
hold on
P = plot([ageom.fr(1),ageom.fl(1),ageom.bl(1),ageom.br(1),ageom.fr(1)],...
     [ageom.fr(2),ageom.fl(2),ageom.bl(2),ageom.br(2),ageom.fr(2)],car_op);
P = [P ; plot(ageom.tfr(1,1:2),ageom.tfr(2,1:2),tyre_opf)];
P = [P ; plot(ageom.tfl(1,1:2),ageom.tfl(2,1:2),tyre_opf)];
P = [P ; plot(ageom.tbr(1,1:2),ageom.tbr(2,1:2),tyre_op)];
P = [P ; plot(ageom.tbl(1,1:2),ageom.tbl(2,1:2),tyre_op)];

end

% ==================================================================================================

% Georg Schildbach, 12/Dec/2013 --- Automobile
% Computes the car circumfence, based on current coordinates and geometric data
% --------------------------------------------------------------------------------------------------
% ageom = autogeometry(x,y,phi,delta,auto)
% --------------------------------------------------------------------------------------------------
% x,y,phi: coordinates of the car (in [m], [m], [rad])
% delta: steering angle [rad]
% auto: structure with geometric data of the car
% --------------------------------------------------------------------------------------------------
% ageom: structure with car geometry data
% --------------------------------------------------------------------------------------------------

function ageom = autogeometry(x,y,phi,delta,auto)

% 1) Linear Constraints ----------------------------------------------------------------------------

% 1.1) Vectors

ageom.gf = [+cos(phi) ; +sin(phi)];
ageom.gb = [-cos(phi) ; -sin(phi)];
ageom.gr = [+sin(phi) ; -cos(phi)];
ageom.gl = [-sin(phi) ; +cos(phi)];

% 1.2) Offsets

ageom.hf =  x*cos(phi) + y*sin(phi) + auto.d + auto.df;
ageom.hb = -x*cos(phi) - y*sin(phi) + auto.db;
ageom.hr =  x*sin(phi) - y*cos(phi) + auto.w/2;
ageom.hl = -x*sin(phi) + y*cos(phi) + auto.w/2;

% 2) Corners ---------------------------------------------------------------------------------------

ageom.fr = [x + (auto.d+auto.df)*cos(phi) ; y + (auto.d+auto.df)*sin(phi)] + auto.w/2*ageom.gr;
ageom.fl = [x + (auto.d+auto.df)*cos(phi) ; y + (auto.d+auto.df)*sin(phi)] + auto.w/2*ageom.gl;
ageom.br = [x - auto.db*cos(phi) ; y - auto.db*sin(phi)] + auto.w/2*ageom.gr;
ageom.bl = [x - auto.db*cos(phi) ; y - auto.db*sin(phi)] + auto.w/2*ageom.gl;

% 3) Tyres -----------------------------------------------------------------------------------------

ageom.sta = [+cos(phi+delta) ; +sin(phi+delta)];
ageom.tfr =  ([x;y]+auto.d*ageom.gf+0.75*auto.w/2*ageom.gr)*[1,1] + ...
                                                   [0.5*auto.tyr*ageom.sta,-0.5*auto.tyr*ageom.sta];
ageom.tfl =  ([x;y]+auto.d*ageom.gf+0.75*auto.w/2*ageom.gl)*[1,1] + ...
                                                   [0.5*auto.tyr*ageom.sta,-0.5*auto.tyr*ageom.sta];
ageom.tbr =  ([x;y]+0.75*auto.w/2*ageom.gr)*[1,1] + [0.5*auto.tyr*ageom.gb,-0.5*auto.tyr*ageom.gb];
ageom.tbl =  ([x;y]+0.75*auto.w/2*ageom.gl)*[1,1] + [0.5*auto.tyr*ageom.gb,-0.5*auto.tyr*ageom.gb];

end

end


%% Attribution
% ME C231A and EECS C220B, UC Berkeley, Fall 2016