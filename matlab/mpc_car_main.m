clear all;
import casadi.*

%% Meta settings
mpc = true;
plot_each_iter = true;
Nsim = 300;
%% Model generation

model_name = 'mpc_car';
model = build_vehicle_model;
model = build_curvilinear_model(model);

%% Sizes
nx = length(model.sym_x);
nu = length(model.sym_u);

%% discretization
N = 50;
T = N; % time horizon length

if mpc
    nlp_solver = 'sqp_rti'; % sqp, sqp_rti
else
    nlp_solver = 'sqp';
end

qp_solver = 'partial_condensing_hpipm';
    % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases, full_condensing_daqp
qp_solver_cond_N = 50; % for partial condensing
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
ocp_model.set('sym_p', model.sym_p);

% cost
ocp_model.set('cost_type','ext_cost');
ocp_model.set('cost_type_e','ext_cost');
ocp_model.set('cost_expr_ext_cost', model.cost_expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e', model.cost_expr_ext_cost_e);

% dynamics
ocp_model.set('dyn_type', 'implicit');
ocp_model.set('dyn_expr_f', model.expr_f_impl);

% constraints
ocp_model.set('constr_type', 'bgh');
ocp_model.set('constr_Jbu', model.constr_Jbu);
ocp_model.set('constr_lbu', model.constr_lbu);
ocp_model.set('constr_ubu', model.constr_ubu);
ocp_model.set('constr_Jbx', model.constr_Jbx);
ocp_model.set('constr_lbx', model.constr_lbx);
ocp_model.set('constr_ubx', model.constr_ubx);
ocp_model.set('constr_expr_h', model.constr_expr_h);
ocp_model.set('constr_lh', model.constr_lh);
ocp_model.set('constr_uh', model.constr_uh);

ocp_model.set('constr_Jbx_0', model.constr_Jbx_0);
ocp_model.set('constr_lbx_0', model.constr_lbx_0);
ocp_model.set('constr_ubx_0', model.constr_ubx_0);


ocp_model.set('constr_Jbx_e', model.constr_Jbx_e);
ocp_model.set('constr_lbx_e', model.constr_lbx_e);
ocp_model.set('constr_ubx_e', model.constr_ubx_e);
ocp_model.set('constr_expr_h_e', model.constr_expr_h_e);
ocp_model.set('constr_lh_e', model.constr_lh_e);
ocp_model.set('constr_uh_e', model.constr_uh_e);

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
%ocp_opts.set('globalization','merit_backtracking');
ocp_opts.set('alpha_min',0.5);
ocp_opts.set('nlp_solver_max_iter', 1000);
%ocp_opts.set('regularize_method','project');
ocp_opts.set('nlp_solver_step_length',1);
%ocp_opts.set('param_scheme','multiple_shooting');
ocp_opts.set('nlp_solver_exact_hessian', 'true');
ocp_opts.set('exact_hess_cost', 0);
ocp_opts.set('exact_hess_dyn', 0);
ocp_opts.set('nlp_solver_tol_stat', 1e-4);
%ocp_opts.set('qp_solver_warm_start', 1);
ocp_opts.set('levenberg_marquardt', 0.01);
ocp_opts.set('param_scheme_N', N);
%ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('ext_fun_compile_flags', '-O2'); % '-O2'

%% create ocp solver
ocp_opts.set('nlp_solver', 'sqp');
ocp = acados_ocp(ocp_model, ocp_opts);
ocp_opts.set('nlp_solver', 'sqp');
mpc_ocp = acados_ocp(ocp_model, ocp_opts);
dt_init = 0.1;
model.s_max = dt_init*N*model.x0_veh(1);
tau = linspace(0,model.s_max,N+1);


x_traj_init = zeros(nx, N+1);
x_traj_init(1,:) = linspace(0,model.s_max,N+1);
x_traj_init(5,:) = linspace(model.x0_veh(1),model.x0_veh(1),N+1);
x_traj_init(4,:) = dt_init;
u_traj_init = zeros(nu, N);
u_traj_init(1,:) = 0.5;
u_traj_init(2,:) = 0.5;
init_delta = model.kappa(x_traj_init(1,:));
init_delta = init_delta.full();
x_traj_init(7,:) = 0.001*init_delta;
x_traj_init(8,:) = 0.001*init_delta;
u_traj_init(3,:) = 0.001*diff(init_delta)/dt_init;

if mpc
    %% Simulation
    x_sim = model.constr_lbx_0;
    u_sim = [];
    t_sim = [0];
    update_vec = logical(ones(nx,1));
    update_vec(4) = false;
    ocp.set('init_x', x_traj_init);
    ocp.set('init_u', u_traj_init);
    ocp.set('init_pi', zeros(nx, N));
    ocp.set('p',[model.s_max]);
    %ocp.solve();
    % get solution for initial conditions.
    %utraj = ocp.get('u');
    %xtraj = ocp.get('x');
    mpc_ocp.set('init_x', x_traj_init);
    mpc_ocp.set('init_u', u_traj_init);
    mpc_ocp.set('init_pi', zeros(nx, N)); 
    mpc_ocp.set('print_level', 3);
    if plot_each_iter
        video = VideoWriter('video.avi');
        video.FrameRate = 5;
        open(video);
    end
    for i=1:Nsim
        mpc_ocp.set('p',[model.s_max]);
        mpc_ocp.solve();
        status = mpc_ocp.get('status');
        if status ~= 0
            % borrowed from acados/utils/types.h
            %statuses = {
            %    0: 'ACADOS_SUCCESS',
            %    1: 'ACADOS_FAILURE',
            %    2: 'ACADOS_MAXITER',
            %    3: 'ACADOS_MINSTEP',
            %    4: 'ACADOS_QP_FAILURE',
            %    5: 'ACADOS_READY'
            % TODO: maybe do an exponential backoff here of the LM constant
            %       if we see QP Failure.
            
            if status == 4
                mpc_ocp.print('stat');
                error(sprintf('acados returned status %d in closed loop iteration %d. Exiting.', status, i));
            else
                warning(sprintf('acados returned status %d in closed loop iteration %d. Exiting.', status, i));
            end
        end
        mpc_ocp.print('stat');
        x0 = mpc_ocp.get('x', 0);
        dt = x0(4);
        u0 = mpc_ocp.get('u', 0);

        u_sim = [u_sim,u0];
        t_sim = [t_sim,t_sim(end)+dt];
        
        x1 = mpc_ocp.get('x', 1);
        ds = x1(1)-x0(1);
        model.s_max = x1(1)+dt_init*N*x1(5);
        x_sim = [x_sim,x1];
        model.constr_lbx_0(update_vec) = x1(update_vec);
        model.constr_ubx_0(update_vec) = x1(update_vec);
        mpc_ocp.set('constr_lbx', model.constr_lbx_0, 0);
        mpc_ocp.set('constr_ubx', model.constr_ubx_0, 0);

        % update init
        x = mpc_ocp.get('x');
        u = mpc_ocp.get('u');

        if plot_each_iter
            ts = [0,cumsum(x(4,:))];
            dt = x(4,1);
            ts = ts(1:end-1);
            [r_x,r_y,r_theta,r_s] = generate_road_curve(model.kappa,0,0,model.s_max);
            f = plot_solution(r_x,r_y,r_theta,r_s,x,u,model,ts,'off');
            frame = getframe(f);
            frame = imresize(frame.cdata,[910,893]);
            writeVideo(video,frame);
            %pause
        end
        % update init
        x = circshift(x, -1, 2);
        x(:,end) = x(:,end-1);
        x(1,end) = model.s_max;
        u = circshift(u, -1, 2);
        u(:,end) = u(:,end-1);
        mpc_ocp.set('init_x', x);
        mpc_ocp.set('init_u', u);
        if x1(1)>model.s_finish
            break;
        end
    end
    close(video);
    %% Plot
    [r_x,r_y,r_theta,r_s] = generate_road_curve(model.kappa,0,0,model.s_finish);
    plot_solution(r_x,r_y,r_theta,r_s,x_sim,u_sim,model,t_sim,'on');
else
    %% call ocp solver
    % set trajectory initialization
    ocp.set('init_x', x_traj_init);
    ocp.set('init_u', u_traj_init);
    ocp.set('init_pi', zeros(nx, N));
    ocp.set('p',[model.s_max]);
    
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
    ts = [0,cumsum(xtraj(4,:))];
    dt = xtraj(4,1);
    ts = ts(1:end-1);
    [r_x,r_y,r_theta,r_s] = generate_road_curve(model.kappa,0,0,model.s_max);
    plot_solution(r_x,r_y,r_theta,r_s,xtraj,utraj,model,ts);
end
