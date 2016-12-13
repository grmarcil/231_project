%% Plot Car and Helper Functions
% Georg Schildbach, 15/Dec/2013 --- Automobile
% Modifications by Greg Marcil, 10/Dec/2016
% Plots the geometric data of the car
% --------------------------------------------------------------------------------------------------
% Plot_Car(z,u,auto,fig,rgb)
% --------------------------------------------------------------------------------------------------
% z: coordinates of the car (x,y,psi,v) ([m], [m], [rad], [m/s])
% u: inputs (effective steering angle, acceleration) ([rad], [m/s^2])
% auto: structure with geometric data of the car
% fig: figure handle
% rgb: red-green-blue value (1x3 vector)
% --------------------------------------------------------------------------------------------------
% P: plot handle (5x1 vector)
% --------------------------------------------------------------------------------------------------

function P = Plot_Car(z,u,auto,fig,rgb)
    % 1) Compute Car Geometry --------------------------------------------------------------------------
    ageom = autogeometry(z(1),z(2),z(3),u(1),auto);

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

    % 3) Plot ------------------------------------------------------------------------------------------

    figure(fig)
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
% ageom = autogeometry(x,y,phi,beta,auto)
% --------------------------------------------------------------------------------------------------
% x,y,phi: coordinates of the car (in [m], [m], [rad])
% beta: effective steering angle [rad]
% auto: structure with geometric data of the car
% --------------------------------------------------------------------------------------------------
% ageom: structure with car geometry data
% --------------------------------------------------------------------------------------------------

function ageom = autogeometry(x,y,phi,beta,auto)

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
    ageom.br = [x - (auto.d+auto.db)*cos(phi) ; y - (auto.d+auto.db)*sin(phi)] + auto.w/2*ageom.gr;
    ageom.bl = [x - (auto.d+auto.db)*cos(phi) ; y - (auto.d+auto.db)*sin(phi)] + auto.w/2*ageom.gl;

    % 3) Tyres -----------------------------------------------------------------------------------------
    delta = atan((1.738 + 1.738)/1.738*tan(beta));
    ageom.sta = [+cos(phi+delta) ; +sin(phi+delta)];
    ageom.tfr =  ([x;y]+auto.d*ageom.gf+0.75*auto.w/2*ageom.gr)*[1,1] + ...
                                                       [0.5*auto.tyr*ageom.sta,-0.5*auto.tyr*ageom.sta];
    ageom.tfl =  ([x;y]+auto.d*ageom.gf+0.75*auto.w/2*ageom.gl)*[1,1] + ...
                                                       [0.5*auto.tyr*ageom.sta,-0.5*auto.tyr*ageom.sta];
    ageom.tbr =  ([x;y]+auto.d*ageom.gb+0.75*auto.w/2*ageom.gr)*[1,1] + [0.5*auto.tyr*ageom.gb,-0.5*auto.tyr*ageom.gb];
    ageom.tbl =  ([x;y]+auto.d*ageom.gb+0.75*auto.w/2*ageom.gl)*[1,1] + [0.5*auto.tyr*ageom.gb,-0.5*auto.tyr*ageom.gb];

end