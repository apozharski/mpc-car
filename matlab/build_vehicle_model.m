function [model] = build_vehicle_model(varargin)
%BUILD_VEHICLE_MODEL Summary of this function goes here
%   Detailed explanation goes here
import casadi.*
if nargin == 1
    unfold_struct(varargin{1},'caller')
end
%% Variable
t_brake = MX.sym('t_brake');
j_brake = MX.sym('j_brake');
t_engine = MX.sym('t_engine');
j_engine = MX.sym('j_engine');
omega_steer = MX.sym('omega_steer');
u = MX.sym('u');
v = MX.sym('v');
omega = MX.sym('omega');
delta = MX.sym('delta');
phi = MX.sym('phi');
%% constants
L_r = 0.5;
L_f = 0.5;
L = L_r+L_f;
k_engine = 1;
k_brake = 1;
brake_bias = 0.5;
mass = 1;
weight_bias = 0.5;
k_r = 1;                 % Rear cornering stiffness
k_f = 1;                 % Front cornering stiffness

%% vehicle model ODEs
% Characteristic angles
%beta = atan2(L_r * tan(delta), L);
slip_f = delta - atan2((v-L_f*omega),u);
slip_r = -atan2((v-L_r*omega),u);

% Forces
F_p_r = k_engine*t_engine-k_brake*brake_bias*t_brake;
F_t_r = k_r*slip_r*(mass*weight_bias);
F_p_f = -k_brake*(1-brake_bias)*t_brake;
F_t_f = k_f*slip_f*(mass*(1-weight_bias));

t_brake_dot = j_brake;
t_engine_dot = j_engine;
delta_dot = omega_steer; % steering rate
u_dot = F_p_r + sin(delta)*F_t_f + cos(delta)*F_p_f;
v_dot = F_t_r + cos(delta)*F_t_f + sin(delta)*F_p_f;
omega_dot = F_t_r*L_r + (cos(delta)*F_t_f + sin(delta)*F_p_f)*L_f;


u_veh = [j_brake;j_engine;omega_steer];
x_veh = [u;v;omega;t_brake;t_engine;delta];
f_veh = [u_dot;v_dot;omega_dot;t_brake_dot;t_engine_dot;delta_dot];

Jbx_veh = [1,0,0,0,0,0
           0,1,0,0,0,0
           0,0,0,1,0,0
           0,0,0,0,1,0
           0,0,0,0,0,1];
lbx_veh = [0;-1;0;0;-1];
ubx_veh = [5;1;1;1;1];

Jbu_veh = eye(3);
lbu_veh = [-1;-1;-0.5];
ubu_veh = [1;1;0.5];

Jbx_e_veh = [0,0,0,1,0,0
             0,0,0,0,1,0];
lbx_e_veh = [0;0];
ubx_e_veh = [1;1];

Jbx0_veh = eye(6);
x0_veh = zeros(6,1);
%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';']);
end
end

