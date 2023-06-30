

%% Example of the frc_racecars in simulation without obstacle avoidance:
%% This example is for the optimal racing of the frc race cars. The model is a simple bicycle model and the lateral acceleration is constraint in order to validate the model assumptions.
%% The simulation starts at s=-2m until one round is completed(s=8.71m). The beginning is cut in the final plots to simulate a 'warm start'. 

clear all
compile = 0;
% close all
import casadi.*
addpath('helper_functions', 'tracks');
addpath('D:\acados\interfaces\acados_matlab_octave')
acados_env_variables_windows
check_acados_requirements()


[trackdata, x0, th0, w] = RaceTrack.loadTrack_02();
track = RaceTrack(trackdata, x0, th0, w);  % x0, th0, w分别定义初始点、初始航向角、道路宽度
Trackdata= [track.dist' track.track_c' track.psi_c' track.curvature'];
cd tracks
save Trackinfo.txt -ascii Trackdata
cd ../
% track_file = 'LMS_Track.txt';
track_file = 'Trackinfo.txt';
[Sref, ~, ~, ~, ~] = getTrack(track_file);

%% Solver parameters
compile_interface = 'auto';
codgen_model = 'true';
nlp_solver = 'sqp_rti'; % sqp, sqp_rti
qp_solver = 'partial_condensing_hpipm';
    % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases
nlp_solver_exact_hessian = 'false'; % false=gauss_newton, true=exact    
qp_solver_cond_N = 5; % for partial condensing
regularize_method = 'no_regularize';
%regularize_method = 'project';
%regularize_method = 'mirror';
%regularize_method = 'convexify';
% integrator type
sim_method = 'erk'; % erk, irk, irk_gnsf

%% horizon parameters
N = 50;
T = 1.0; % time horizon length,这样计算步长就是det_T = T/N

%% model dynamics

[model, constraint] = bicycle_model(track_file);

nx = length(model.x);
nu = length(model.u);

%% model to create the solver
ocp_model = acados_ocp_model();

%% acados ocp model
ocp_model.set('name', model.name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x', model.x);
ocp_model.set('sym_u', model.u);
ocp_model.set('sym_xdot', model.xdot);
%ocp_model.set('sym_z', model.z);
%ocp_model.set('sym_p', model.p);

% dynamics
if (strcmp(sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.f_expl_expr);
else % irk irk_gnsf
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.f_impl_expr);
end

% constraintsJbx = zeros(1,nx);
nbx = 4;
Jbx = zeros(nbx,nx); %定义状态变量的选择矩阵 d_psi, vx, vy, e_psi, e_y, s, delta, Tc
Jbx(1,5) = 1;                                       %e_y
Jbx(2,2) = 1;                                       %Track_vel
Jbx(3,7) = 1;                                       %delta
Jbx(4,8) = 1;                                       %Tc
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', [model.ey_min, model.Track_vel_min , model.delta_min, model.Tc_min]);
ocp_model.set('constr_ubx', [model.ey_max, model.Track_vel_max , model.delta_max, model.Tc_max] );

nbu = 2;
Jbu = zeros(nbu,nu);
Jbu(1,1) = 1;                                       %d_delta
Jbu(2,2) = 1;                                       %d_Tc
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', [ model.ddelta_min, model.dTc_min]);
ocp_model.set('constr_ubu', [ model.ddelta_max, model.dTc_max]);

%ocp_model.set('constr_type', 'bgh');
nh = 2;
ocp_model.set('constr_expr_h', constraint.expr);
ocp_model.set('constr_lh', [...
                                constraint.alon_min,...
                                constraint.alat_min,...
                           ]);
ocp_model.set('constr_uh', [...
                                constraint.alon_max,...
                                constraint.alat_max,...
                           ]);    
%ocp_model.set('constr_expr_h_e', constraint.expr);     
%ocp_model.set('constr_lh_e', 0);
%ocp_model.set('constr_uh_e', 0);

% Configure constraint slack variables
nsh = 2;
Jsh = zeros(nh, nsh);
Jsh(1,1) = 1;
Jsh(2,2) = 1;
ocp_model.set('constr_Jsh', Jsh);
% Set cost on slack
% L1 slack (linear term)
ocp_model.set('cost_zl', 100 * ones(nsh,1));
ocp_model.set('cost_zu', 100 * ones(nsh,1));
% L2 slack (squared term)
ocp_model.set('cost_Zl', 0 * ones(nsh,nsh));
ocp_model.set('cost_Zu', 0 * ones(nsh,nsh));

% set intial condition  %初始值
ocp_model.set('constr_x0', model.x0);

% cost = define linear cost on x and u
%ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
%ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);

ocp_model.set('cost_type', 'linear_ls');
ocp_model.set('cost_type_e', 'linear_ls');

% number of outputs is the concatenation of x and u
ny = nx + nu;
ny_e = nx;

% The linear cost contributions is defined through Vx, Vu and Vz
Vx = zeros(ny, nx);
Vx_e = zeros(ny_e, nx);
Vu = zeros(ny, nu);

Vx(1:nx,:) = eye(nx);
Vx_e(1:nx,:) = eye(nx);
Vu(nx+1:end,:) = eye(nu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_Vu', Vu);

% Define cost on states and input
% Q = diag([ 1e-1, 1e-4, 1e-8, 1e-8, 1e-3, 5e-3 ]); s, n, alpha, v, D, delta
%          d_psi,  vx,     vy,    e_psi,   e_y,      s,      delta,  T
Q = diag([ 1e-8,   -1e-1,  1e-4,  1e-4,    10e-3,    1e-1,   5e-3,   1e-3 ]);  
R = eye(nu);
R(1, 1) = 1e-3;
R(2, 2) = 1e-3;

% Qe = diag([5e0, 1e1, 1e-8, 1e-8, 5e-3, 2e-3 ]);
Qe = diag([ 1e-8, -10e-1,   1e-3,  1e-3,     1e1,     1e0,   5e-3,   1e-3]);

unscale = N / T;
W = unscale * blkdiag(Q, R);
W_e = Qe / unscale*10;
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);

% set intial references
y_ref = zeros(ny,1);
y_ref_e =  zeros(ny_e,1);
y_ref(6) = 0; % set reference on 's' to 1 to push the car forward (progress)  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% i修改为0
ocp_model.set('cost_y_ref', y_ref);
ocp_model.set('cost_y_ref_e', y_ref_e);

% ... see ocp_model.model_struct to see what other fields can be set

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
%ocp_opts.set('compile_interface', compile_interface);
%ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian); 
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('sim_method_num_stages', 4);  % Use 4th order ODE solver
ocp_opts.set('sim_method_num_steps', 3);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_iter_max', 300);
%ocp_opts.set('regularize_method', regularize_method);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('nlp_solver_tol_stat', 1e-4);
ocp_opts.set('nlp_solver_tol_eq', 1e-4);
ocp_opts.set('nlp_solver_tol_ineq', 1e-4);
ocp_opts.set('nlp_solver_tol_comp', 1e-4);
% ... see ocp_opts.opts_struct to see what other fields can be set

%% go embedded
% % % to generate templated C code
%% get available simulink_opts with default options
%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);
if compile

    simulink_opts = get_acados_simulink_opts;

    % manipulate simulink_opts
    
    % inputs
    simulink_opts.inputs.cost_W_0 = 0;
    simulink_opts.inputs.cost_W = 0;
    simulink_opts.inputs.cost_W_e = 0;
    simulink_opts.inputs.x_init = 0;
    
    % outputs
    simulink_opts.outputs.utraj = 0;
    simulink_opts.outputs.xtraj = 1;
    simulink_opts.outputs.cost_value = 1;
    
    simulink_opts.samplingtime = '-1';
        % 't0' (default) - use time step between shooting node 0 and 1
        % '-1' - inherit sampling time from other parts of simulink model
    
    % set time step for code generated integrator - default is length of first
    % time step of ocp object
    ocp.opts_struct.Tsim = 0.02;


%% Render templated Code for the model contained in ocp object
    ocp.generate_c_code(simulink_opts);
    cd c_generated_code
    make_sfun; % ocp solver
    make_sfun_sim; % integrator
    %% Copy Simulink example blocks into c_generated_code
    source_folder = pwd;
    target_folder = fullfile(pwd, '..');
    copyfile( fullfile(source_folder, 'acados_sim_solver_sfunction_bicycle_model.mexw64'), target_folder );
    copyfile( fullfile(source_folder, 'acados_solver_sfunction_bicycle_model.mexw64'), target_folder );
    cd ../
end
Tf = 15.00;  % maximum simulation time[s]
open_system( 'simulink_model_closedloop')
% open_system(fullfile(target_folder, 'simulink_model_closedloop_all'))
%%
disp('Press play in Simulink!');
plotted