function plot_solution(r_x,r_y,r_theta,r_s,v_traj,v_u)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
close all;
%% Plot vehicle Curvilinear state
ts = [0,cumsum(v_traj(4,:))];
ts = ts(1:end-1);
figure;
subplot(2,2,1);
plot(ts,v_traj(1,:));
ylabel('$s$','Interpreter','latex');
subplot(2,2,2);
plot(ts,v_traj(2,:));
ylabel('$n$','Interpreter','latex');
subplot(2,2,3);
plot(ts,v_traj(3,:));
ylabel('$\alpha$','Interpreter','latex');

%% Plot vehicle CG state

figure;
subplot(2,2,1);
plot(ts,v_traj(5,:));
ylabel('$u$','Interpreter','latex');
subplot(2,2,2);
plot(ts,v_traj(6,:));
ylabel('$v$','Interpreter','latex');
subplot(2,2,3);
plot(ts,v_traj(7,:));
ylabel('$\omega$','Interpreter','latex');
subplot(2,2,4);
plot(ts,v_traj(10,:));
ylabel('$\delta$','Interpreter','latex');

figure;
subplot(2,1,1);
plot(ts,v_traj(8,:));
ylabel('$t_{\mathrm{brake}}$','Interpreter','latex');
subplot(2,1,2);
plot(ts,v_traj(9,:));
ylabel('$t_{\mathrm{engine}}$','Interpreter','latex');
%% Plot Controls
figure;
subplot(2,2,1);
stairs(ts(1:end-1),v_u(1,:));
ylabel('$j_{\textrm{brake}}$','Interpreter','latex')
subplot(2,2,2);
stairs(ts(1:end-1),v_u(2,:));
ylabel('$j_{\textrm{engine}}$','Interpreter','latex')
subplot(2,2,3);
stairs(ts(1:end-1),v_u(3,:));
ylabel('$\omega_{\textrm{steer}}$','Interpreter','latex')
%% Plot road trajectory
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