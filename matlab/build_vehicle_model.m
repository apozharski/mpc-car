function [model] = build_vehicle_model(varargin)
%BUILD_VEHICLE_MODEL Summary of this function goes here
%   Detailed explanation goes here
import casadi.*
if nargin == 1
    unfold_struct(varargin{1},'caller')
end
%% Variables
% State
t_brake = MX.sym('t_brake');
t_engine = MX.sym('t_engine');
omega_steer = MX.sym('omega_steer');
u = MX.sym('u');
v = MX.sym('v');
omega = MX.sym('omega');
delta = MX.sym('delta');
dt = MX.sym('dt');

% dots
t_brake_dot = MX.sym('t_brake_dot');
t_engine_dot = MX.sym('t_engine_dot');
omega_steer_dot = MX.sym('omega_steer_dot');
u_dot = MX.sym('u_dot');
v_dot = MX.sym('v_dot');
omega_dot = MX.sym('omega_dot');
delta_dot = MX.sym('delta_dot');

% control
j_brake = MX.sym('j_brake');
j_engine = MX.sym('j_engine');
%% constants
L_r = 1.482;
L_f = 1.118;
L = L_r+L_f;
k_engine = 1000;
k_brake = 2000;
brake_bias = 0.5;
mass = 1440;
I = 1730;
weight_bias = 0.5;
k_r = 29;                 % Rear cornering stiffness
k_f = 29;                 % Front cornering stiffness

C_u = 0.39;
C_v = 0.39;

%% vehicle model ODEs
% Characteristic angles
%beta = atan2(L_r * tan(delta), L);
slip_f = atan2((v+L_f*omega),u) - delta;
slip_r = atan2((v-L_r*omega),u);

% Forces
F_p_r = k_engine*t_engine-k_brake*brake_bias*t_brake;
F_t_r = -k_r*slip_r*(mass*weight_bias);
F_p_f = -k_brake*(1-brake_bias)*t_brake;
F_t_f = -k_f*slip_f*(mass*(1-weight_bias));

f_t_brake_dot = j_brake;
f_t_engine_dot = j_engine;
f_delta_dot = omega_steer; % steering rate
f_u_dot = F_p_r - sin(delta)*F_t_f + cos(delta)*F_p_f - C_u*u^2;
f_v_dot = F_t_r + cos(delta)*F_t_f + sin(delta)*F_p_f - C_v*v^2;
f_omega_dot = -F_t_r*L_r + (cos(delta)*F_t_f + sin(delta)*F_p_f)*L_f;


u_veh = [t_brake;t_engine;omega_steer];
x_veh = [u;v;omega;delta];
x_dot_veh = [u_dot;v_dot;omega_dot;delta_dot];
f_veh = [u_dot - dt*f_u_dot/mass
         v_dot - dt*f_v_dot/mass
         omega_dot - dt*f_omega_dot/I
         delta_dot - dt*f_delta_dot];

Jbx_veh = [0,0,0,1];
lbx_veh = [-pi/4];
ubx_veh = [pi/4];

Jbu_veh = eye(3);
lbu_veh = [0;0;-0.5];
ubu_veh = [1;1;0.5];

Jbx_e_veh = [];
lbx_e_veh = [];
ubx_e_veh = [];

Jbx0_veh = eye(4);
x0_veh = zeros(4,1);
x0_veh(1) = 1e-10;
%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';']);
end
end

