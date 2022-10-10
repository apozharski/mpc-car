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
method = 'erk'; % irk, irk_gnsf
model_name =  'sim_car';

% simulation parameters
N_sim = 100;
h = 1; % simulation time
x0 = [0; 0; 0;xtraj(4,1)]; % initial state
u0 = [1,0]; % control input

%% acados sim model
sim_model = acados_sim_model();
sim_model.set('name', model_name);
sim_model.set('T', h);

sim_model.set('sym_x', model.sym_x);
sim_model.set('sym_u', model.sym_u);

sim_model.set('dyn_type', 'explicit');
sim_model.set('dyn_expr_f', model.expr_f_expl);

%% acados sim opts
sim_opts = acados_sim_opts();

sim_opts.set('compile_interface', compile_interface);
sim_opts.set('num_stages', 2);
sim_opts.set('num_steps', 3);
sim_opts.set('newton_iter', 2); % for implicit intgrators
sim_opts.set('method', method);


%% create integrator
sim = acados_sim(sim_model, sim_opts);


%% simulate system in loop
x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0;

for ii=1:N_sim
	
	% set initial state
	sim.set('x', x_sim(:,ii));
	sim.set('u', utraj(:,ii));
    %sim.set('u', u0);
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
end
%% plot
close all;
ts = [0,cumsum(xtraj(4,:))];
ts = ts(1:end-1);
figure;
plot(ts,x_sim(1,:));
ylabel('s');
figure;
plot(ts,x_sim(2,:));
ylabel('n');
figure;
plot(ts,x_sim(3,:));
ylabel('alpha');
figure;
stairs(ts(1:end-1),utraj(1,:));
figure;
stairs(ts(1:end-1),utraj(2,:));