function [x_pos,y_pos,theta_pos,s] = generate_road_curve(kappa,x0,y0,s_max)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

import casadi.*

x = MX.sym('x');
y = MX.sym('y');
theta = MX.sym('theta');
s = MX.sym('s');

f = [cos(theta);sin(theta);kappa(s);1];

grid = linspace(0,s_max,1001);
r = zeros(4,length(grid));

ode = struct('x', [x;y;theta;s], 'ode', f);
%opts = struct('grid',grid,'output_t0', true, 'print_stats', true);
opts = struct('tf',grid(2)-grid(1));
F = integrator('F','rk',ode,opts);
for k=1:length(grid)-1
     next = F('x0',r(:,k));
     r(:,k+1) = full(next.xf);
end
%r = F('x0',[x0;y0;0]);
xf = full(r);
x_pos = xf(1,:);
y_pos = xf(2,:);
theta_pos = xf(3,:);
s = grid;
end