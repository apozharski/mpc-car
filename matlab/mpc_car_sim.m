import casadi.*

%% Model generation

model_name = 'mpc_car';
model = build_vehicle_model;
model = build_curvilinear_model(model);

%% Sizes
nx = length(model.sym_x);
nu = length(model.sym_u);

%% arguments
compile_interface = 'auto';
method = 'irk'; % irk, irk_gnsf
model_name =  'sim_car';

% simulation parameters
N_sim = 1000;
h = 1; % simulation time
x0 = [0; 0; 0; 0.01; 5; 0; 0; 0; 0; 0.0]; % initial state
%x0 = [0; 0; 0;xtraj(4,1);0]; % initial state
u0 = [1,0.01;]; % control inputs

%% acados sim model
sim_model = acados_sim_model();
sim_model.set('name', model_name);
sim_model.set('T', h);

sim_model.set('sym_x', model.sym_x);
sim_model.set('sym_xdot',model.sym_xdot)
sim_model.set('sym_u', model.sym_u);

sim_model.set('dyn_type', 'implicit');
sim_model.set('dyn_expr_f', model.expr_f_impl);

%% acados sim opts
sim_opts = acados_sim_opts();

sim_opts.set('compile_interface', compile_interface);
sim_opts.set('num_stages', 2);
sim_opts.set('num_steps', 3);
sim_opts.set('newton_iter', 3); % for implicit intgrators
sim_opts.set('method', method);


%% create integrator
sim = acados_sim(sim_model, sim_opts);


%% simulate system in loop
x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0;
u_sim = zeros(nu, N_sim);
t = 0;
dt = x_sim(4,1);
j_engine = interpolant('j_engine','linear',{[0,1,2,3,4,5,6,7]},[0,0,0,0,0,0,0,0]);
%j_engine = interpolant('j_engine','linear',{[0,1,2,3,4,5,6,7]},[0,0,0,0,0,0,0,0]);
j_brake = interpolant('j_engine','linear',{[0,1,2,3,4,5,6,7]},[0,0,0,0,0,0,0,0]);
%j_brake = interpolant('j_engine','linear',{[0,1,2,3,4,5,6,7]},[0,0,0,0,0,0,0,0]);
omega_steer = interpolant('j_engine','linear',{[0,1,2,3,4,5,6,7]},[0,0.1,0,-0.1,-0.1,0,0,0]);
%omega_steer = interpolant('j_engine','linear',{[0,1,2,3,4,5,6,7]},[0,0,0,0,0,0,0,0]);
for ii=1:N_sim
	% set initial state
	sim.set('x', x_sim(:,ii));
    u = [j_brake(t),j_engine(t),omega_steer(t)];
    u_sim(:,ii) = u.full();
	sim.set('u', u.full());
    %sim.set('u', [1,0.03]);
    % initialize implicit integrator
    if (strcmp(method, 'irk'))
        sim.set('xdot', zeros(nx,1));
    elseif (strcmp(method, 'irk_gnsf'))
        n_out = sim.model_struct.dim_gnsf_nout;
        sim.set('phi_guess', zeros(n_out,1));
    end

	% solve
	sim.solve();
	% get simulated state
	x_sim(:,ii+1) = sim.get('xn');
    t = t+dt;
end
%% plot
[r_x,r_y,r_theta,r_s] = generate_road_curve(model.kappa,0,0,model.s_max);
plot_solution(r_x,r_y,r_theta,r_s,x_sim,u_sim);
