import casadi.*

%% Model generation

model_name = 'mpc_car';
model = build_vehicle_model;
model = build_curvilinear_model(model)

%% Acados OCP setup (TODO)