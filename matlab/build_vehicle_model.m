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

%% vehicle model ODEs
beta = atan2(L_r * tan(delta), L);
slip_f = delta - (v-L_f*omega)/u;
slip_r = -(v-L_r*omega)/u;

t_brake_dot = j_brake;
t_engine_dot = j_engine;
delta_dot = omega_steer; % steering rate
u_dot = (k_engine*t_engine-k_brake*t_brake)*cos(beta);
v_dot = (k_engine*t_engine-k_brake*t_brake)*sin(beta);
omega_dot = (k_engine*t_engine-k_brake*t_brake)*tan(delta)*cos(beta)/L;


u_veh = [j_brake;j_engine;omega_steer];
x_veh = [u;v;omega;t_brake;t_engine;delta];
f_veh = [u_dot;v_dot;omega_dot;t_brake_dot;t_engine_dot;delta_dot];

Jbx_veh = [0,0,0,0,0,1
           1,0,0,0,0,0
           0,0,0,1,0,0
           0,0,0,0,1,0];
lbx_veh = [-1;0;0;0];
ubx_veh = [1;5;1;1];

Jbu_veh = eye(3);
lbu_veh = [-1;-1;-0.5];
ubu_veh = [1;1;0.5];

Jbx_e_veh = [];
lbx_e_veh = [];
ubx_e_veh = [];

Jbx0_veh = eye(6);
x0_veh = zeros(6,1);
%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end

