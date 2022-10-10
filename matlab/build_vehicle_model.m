function [model] = build_vehicle_model(varargin)
%BUILD_VEHICLE_MODEL Summary of this function goes here
%   Detailed explanation goes here
import casadi.*
if nargin == 1
    unfold_struct(varargin{1},'caller')
end
%% Variable
v_wheel = MX.sym('v_wheel');
delta = MX.sym('delta');
phi = MX.sym('phi');
%% constants
L   = 1;
l_r = 0.5;
%% vehicle model ODEs
beta = atan2(l_r * tan(delta), L);
u = v_wheel * cos(beta);
v = v_wheel * sin(beta);
omega =  v_wheel * tan(delta) * cos(beta) / L;
delta_dot = phi; % steering rate


u_veh = [v_wheel;phi];
x_veh = [delta];
f_veh = [phi];

Jbx_veh = [1];
lbx_veh = [-1];
ubx_veh = [1];

Jbu_veh = eye(2);
lbu_veh = [0;-0.1];
ubu_veh = [1;0.1];

Jbx_e_veh = [];
lbx_e_veh = [];
ubx_e_veh = [];

Jbx0_veh = [1];
x0_veh = [0];
%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end

