%
% Copyright 2023

function [model, constraint] = bicycle_model(track)

import casadi.*
model = struct();
constraint = struct();

model_name = 'bicycle_model';
[~, ~, ~, w] = RaceTrack.loadTrack_01();
% load track parameters
[s0, ~, ~, ~, kapparef] = getTrack(track);

pathlength = s0(end);
% copy loop to beginning and end
s0 = [s0; s0(end) + s0(2:end)];
kapparef = [kapparef; kapparef(2:end)];
s0 = [-s0(end-1) + s0(end-80 : end-2); s0];
kapparef = [kapparef(end-79 : end-1); kapparef];

% compute spline interpolations
kapparef_s = interpolant('kapparef_s', 'bspline', {s0}, kapparef);

%% Race car parameters
m    = 500;      % vehicle mass
Iz  =  600 ;     % vehicle moment of inertia (yaw axis)
g    = 9.81;     % gravitation
l_f  = 0.9 ;     % distance of the front wheel to the center of mass 
l_r  = 1.5-0.5;      % distance of the rear wheel to the center of mass
L    =l_f+l_r;

%% Pacejka lateral dynamics parameters
B_f = 0.4;              % stiffnes factor (Pacejka) (front wheel)
C_f = 8;                % shape factor (Pacejka) (front wheel)
D_f = -4560.4;           % peak value (Pacejka) (front wheel)
E_f = -0.5;             % curvature factor (Pacejka) (front wheel)
B_r = 0.45;             % stiffnes factor (Pacejka) (rear wheel)
C_r = 8;                % shape factor (Pacejka) (rear wheel)
D_r = -4000;             % peak value (Pacejka) (rear wheel)
E_r = -0.5;             % curvature factor (Pacejka) (rear wheel)
%% CasADi Model
% set up states & controls
d_psi = MX.sym('d_psi');
vx = MX.sym(' vx ');
vy = MX.sym('vy ');
e_psi = MX.sym('e_psi');
e_y = MX.sym(' e_y ');
s = MX.sym('s ');                    % track centerline progress
delta = MX.sym('delta  ');
Tc = MX.sym('Tc  ');                   %wheel torque gain control signal,  -1=max.braking, 1=max acc.
x = vertcat(d_psi, vx, vy, e_psi, e_y, s, delta, Tc);

% controls
der_delta = MX.sym('der_delta ');
der_Tc  = MX.sym('der_Tc ');
u = vertcat( der_delta, der_Tc );

% xdot
d_d_psi = MX.sym('d_d_psi');
d_vx = MX.sym('d_vx');
d_vy = MX.sym('d_vy');
d_e_psi = MX.sym('d_e_psi');
d_e_y = MX.sym('d_e_y');
d_s = MX.sym(' d_s');
d_delta = MX.sym(' d_delta ');
d_Tc = MX.sym('d_Tc');
xdot = vertcat(d_d_psi, d_vx, d_vy, d_e_psi, d_e_y, d_s, d_delta, d_Tc);

% algebraic variables
z = [];

% parameters
p = [];
%--------------------------------------------------------------
% Wheel slip angles (slip ration not being used for now)
a_r = atan2(vy-l_r*d_psi,vx);%*(vx>1);
a_f = (atan2(vy+l_f*d_psi,vx) - delta);%*(vx>1);

%--------------------------------------------------------------
% Tyre forces
% desired total wheel torque to be applied
% maxbrakeWForce = 8000; % = 2*g*M;  % allow ~ 2g brake
maxmotorWForce = 4000 ;% = 1*g*M;  % allow ~ 1g acc   ~ 1g brake
totalWForce = Tc * maxmotorWForce;%( (Tc>0)*maxmotorWForce+(Tc<0)*maxbrakeWForce ); % % (NOT DIFFERENTIABLE)  *sign(vx) 
% longitudinal wheel torque distribution
zeta = 0.5;

% Wheel forces in wheel coordinates (z-axis points down, x-axis front)
% This means positive W_Fy_f turns vehicle to the right
Fx_r = zeta * totalWForce;
Fx_f = (1-zeta) * totalWForce;

Fy_r = D_r*sin(C_r*atan(B_r*a_r-E_r*(B_r*a_r -atan(B_r*a_r)))); % rear lateral force
Fy_f = D_f*sin(C_f*atan(B_f*a_f-E_f*(B_f*a_f -atan(B_f*a_f)))); % front lateral force
s_dot  = (vx*cos(e_psi) - vy*sin(e_psi))/(1 - kapparef_s(s) * e_y);
f_expl = vertcat(...
      (l_f*Fy_f*cos(delta) +Fx_f*l_f*sin(delta) - l_r*Fy_r)/Iz,...                           %yaw__dotdot
      (Fx_f*cos(delta) + Fx_r - Fy_f*sin(delta))/m + vy*d_psi,...                            %vx__dot
      (Fy_f*cos(delta) + Fy_r + Fx_f*sin(delta))/m - vx*d_psi,...                            %vy__dot
      d_psi -  kapparef_s(s)*s_dot ,...                                                      %e_psi__dot
      vy*cos(e_psi) + vx*sin(e_psi),...                                                      %e_y__dot
      s_dot ,...                                                                             %d_Track_vel
      der_delta,...
      der_Tc...
);

% constraint on forces
a_lat = (Fy_f*cos(delta) + Fy_r + Fx_f*sin(delta))/m;
a_lon =(Fx_f*cos(delta) + Fx_r - Fy_f*sin(delta))/m;

% Model bounds
model.ey_min = -w/2;  % width of the track [m]
model.ey_max = w/2;  % width of the track [m]

% state bounds

model.delta_min = -0.40;  % minimum steering angle [rad]
model.delta_max =  0.40;  % maximum steering angle [rad]
model.Tc_min = -1;  % gas pedal [%]
model.Tc_max =  1;  % gas pedal [%]
model.Track_vel_min = 1;  % minimum centerline track velocity [m/s]
model.Track_vel_max = 30;  % maximum centerline track velocity [m/s]

% input bounds
model.ddelta_min = -2.0;  % minimum change rate of stering angle [rad/s]
model.ddelta_max =  2.0;  % maximum change rate of steering angle [rad/s]
model.dTc_min = -4;  % -10.0  % minimum throttle change rate
model.dTc_max =  4;  % 10.0  % maximum throttle change rate

% nonlinear constraint
constraint.alat_min = -4;  % maximum lateral force [m/s^2]
constraint.alat_max =  4;  % maximum lateral force [m/s^1]

constraint.alon_min = -8;  % maximum longitudinal force [m/s^2]
constraint.alon_max =  8;  % maximum longitudinal force [m/s^2]

% Define initial conditions
model.x0 = [0,2, 0, 0, 0, 0, 0,0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   -2修改为0

% define nonlinear constraints struct
constraint.alat = Function('a_lat', {x, u}, {a_lat});
constraint.alon = Function('a_lon', {x, u}, {a_lon});
constraint.pathlength = pathlength;
constraint.expr = vertcat(a_lon, a_lat); %定义需要约束的状态量

% Define model struct
params = struct();
params.B_f = B_f;
params.C_f = C_f;
params.D_f = D_f;
params.E_f = E_f;
params.B_r = B_r;
params.C_r = C_r;
params.C_r = D_r;
params.C_r = E_r;

model.f_impl_expr = f_expl - xdot; %隐式模型表达
model.f_expl_expr = f_expl;        %显式模型表达
model.x = x;
model.xdot = xdot;
model.u = u;
model.z = z;
model.p = p;
model.name = model_name;
model.params = params;

end
