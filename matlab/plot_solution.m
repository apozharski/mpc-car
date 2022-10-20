function plot_solution(r_x,r_y,r_theta,r_s,v_traj)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

% Break out vehicle 
v_s = v_traj(1,:);
v_n = v_traj(2,:);
v_alpha = v_traj(3,:);
v_u = v_traj(5,:);
v_v = v_traj(6,:); 
v_delta = v_traj(10,:);

% Calculate left and right hand side
r_lx = r_x - sin(r_theta);
r_ly = r_y + cos(r_theta);
r_rx = r_x + sin(r_theta);
r_ry = r_y - cos(r_theta);

% Calculate vehicle poses
v_s(end) = 25; %hack to fix nans
v_theta = interp1(r_s,r_theta,v_s);
v_x = interp1(r_s,r_x,v_s) - v_n.*sin(v_theta);
v_y = interp1(r_s,r_y,v_s) + v_n.*cos(v_theta);

% Calculate velocities
rotation = exp((v_theta+v_alpha)*1i);
v_uv_rotated = rotation .* ([1 1i]*[v_u;v_v]);
v_u_rotated = real(v_uv_rotated);
v_v_rotated = imag(v_uv_rotated);

figure;
hold on;
plot(r_x,r_y, 'b');
plot(r_lx,r_ly, 'r');
plot(r_rx,r_ry, 'r');
plot(v_x,v_y,'g');
axis equal;
quiver(v_x,v_y,v_u_rotated,v_v_rotated);
end