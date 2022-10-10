function [model] = build_vehicle_model(varargin)
%BUILD_VEHICLE_MODEL Summary of this function goes here
%   Detailed explanation goes here
import casadi.*
if nargin == 1
    unfold_struct(varargin{1},'caller')
end
%% Variable 
u = MX.sym('u');
omega = MX.sym('omega');
u_veh = [u;omega];
v = 0;
x_veh = [];
f_veh = [];

Jbx_veh = [];
lbx_veh = [];
ubx_veh = [];

Jbu_veh = eye(2);
lbu_veh = [0;-1];
ubu_veh = [1;1];

Jbx_e_veh = [];
lbx_e_veh = [];
ubx_e_veh = [];

Jbx0_veh = [];
x0_veh = [];
%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end

