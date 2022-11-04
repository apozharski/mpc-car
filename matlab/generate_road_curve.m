function [x_pos,y_pos,theta_pos,s] = generate_road_curve(kappa,x0,y0,s_max)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

import casadi.*

x = MX.sym('x');
y = MX.sym('y');
theta = MX.sym('theta');
s = MX.sym('s');

f = [cos(theta);sin(theta);kappa(s)];

grid = linspace(0,s_max,201);

ode = struct('x', [x;y;theta], 't', s, 'ode', f);
opts = struct('grid',grid,'output_t0', true);
F = integrator('F','cvodes',ode,opts);
r = F('x0',[x0;y0;0]);
xf = full(r.xf);
x_pos = xf(1,:);
y_pos = xf(2,:);
theta_pos = xf(3,:);
s = grid;
end