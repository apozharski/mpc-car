clear all;
import casadi.*

%% Model generation

model_name = 'mpc_car';
model = build_vehicle_model;
model = build_curvilinear_model(model);

%% Sizes
nx = length(model.sym_x);
nu = length(model.sym_u);

%% discretization
N = 100;
T = N; % time horizon length

nlp_solver = 'sqp'; % sqp, sqp_rti
qp_solver = 'partial_condensing_hpipm';
    % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases, full_condensing_daqp
qp_solver_cond_N = 100; % for partial condensing
% integrator type
sim_method = 'irk'; % erk, irk, irk_gnsf

%% model to create the solver
ocp_model = acados_ocp_model();

%% acados ocp model
ocp_model.set('name', model_name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

% cost
ocp_model.set('cost_type','ext_cost');
ocp_model.set('cost_type_e','ext_cost');
ocp_model.set('cost_expr_ext_cost', model.cost_expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e', model.cost_expr_ext_cost_e);

% dynamics
% if (strcmp(sim_method, 'erk'))
    %ocp_model.set('dyn_type', 'explicit');
    %ocp_model.set('dyn_expr_f', model.expr_f_expl);
% else % irk irk_gnsf
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
% end

% constraints
% ocp_model.set('constr_type', 'bgh');
% ocp_model.set('constr_Jbu',eye(nu));
% ocp_model.set('constr_lbu', model.constr_lbu);
% ocp_model.set('constr_ubu', model.constr_ubu);
% ocp_model.set('constr_Jbx',eye(nx));
% ocp_model.set('constr_lbx', model.constr_lbx);
% ocp_model.set('constr_ubx', model.constr_ubx);
% 

ocp_model.set('constr_type', 'bgh');
ocp_model.set('constr_Jbu', model.constr_Jbu);
ocp_model.set('constr_lbu', model.constr_lbu);
ocp_model.set('constr_ubu', model.constr_ubu);
ocp_model.set('constr_Jbx', model.constr_Jbx);
ocp_model.set('constr_lbx', model.constr_lbx);
ocp_model.set('constr_ubx', model.constr_ubx);

ocp_model.set('constr_Jbx_0', model.constr_Jbx_0);
ocp_model.set('constr_lbx_0', model.constr_lbx_0);
ocp_model.set('constr_ubx_0', model.constr_ubx_0);
%ocp_model.set('constr_x0', [0;0;0;0.1]);

ocp_model.set('constr_Jbx_e', model.constr_Jbx_e);
ocp_model.set('constr_lbx_e', model.constr_lbx_e);
ocp_model.set('constr_ubx_e', model.constr_ubx_e);
% ... see ocp_model.model_struct to see what other fields can be set


%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('globalization','merit_backtracking');
ocp_opts.set('nlp_solver_max_iter', 5000);
ocp_opts.set('regularize_method','mirror');
%ocp_opts.set('param_scheme','multiple_shooting');
%ocp_opts.set('nlp_solver_exact_hessian', 'true');
%ocp_opts.set('exact_hess_cost', 'false');
ocp_opts.set('nlp_solver_tol_stat', 1e-5);
ocp_opts.set('qp_solver_warm_start', 1);
ocp_opts.set('levenberg_marquardt', 0.1);
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('ext_fun_compile_flags', ''); % '-O2'
% ... see ocp_opts.opts_struct to see what other fields can be set

%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);
dt_init = 3;
tau = linspace(0,100,N+1);

x_traj_init = zeros(nx, N+1);
x_traj_init(1,:) = linspace(0,model.s_max,N+1);
x_traj_init(5,:) = linspace(1e-10,10,N+1);
x_traj_init(4,:) = dt_init;
u_traj_init = zeros(nu, N);
u_traj_init(2,:) = 1;
init_delta = model.kappa(x_traj_init(1,:));
init_delta = init_delta.full();
x_traj_init(7,:) = init_delta;
x_traj_init(8,:) = init_delta;
u_traj_init(3,:) = diff(init_delta)/dt_init;
%% call ocp solver

% set trajectory initialization
ocp.set('init_x', x_traj_init);
ocp.set('init_u', u_traj_init);
ocp.set('init_pi', zeros(nx, N))

% change values for specific shooting node using:
%   ocp.set('field', value, optional: stage_index)
%ocp.set('constr_lbx', x0, 0)
ocp.set('print_level', 3);
% solve
ocp.solve();
% get solution
utraj = ocp.get('u');
xtraj = ocp.get('x');

status = ocp.get('status'); % 0 - success
ocp.print('stat')
%% Plot
[r_x,r_y,r_theta,r_s] = generate_road_curve(model.kappa,0,0,model.s_max);
plot_solution(r_x,r_y,r_theta,r_s,xtraj,utraj,model);