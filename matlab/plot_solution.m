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
ylabel('$s$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
plot(ts,v_traj(2,:));
ylabel('$n$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
plot(ts,v_traj(3,:));
ylabel('$\alpha$','Interpreter','latex','fontsize', 30);

%% Plot vehicle CG state

figure;
subplot(2,2,1);
plot(ts,v_traj(5,:));
ylabel('$u$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
plot(ts,v_traj(6,:));
ylabel('$v$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
plot(ts,v_traj(7,:));
ylabel('$\omega$','Interpreter','latex','fontsize', 30);
subplot(2,2,4);
plot(ts,v_traj(10,:));
ylabel('$\delta$','Interpreter','latex','fontsize', 30);

figure;
subplot(2,1,1);
plot(ts,v_traj(8,:));
ylabel('$t_{\mathrm{brake}}$','Interpreter','latex','fontsize', 30);
subplot(2,1,2);
plot(ts,v_traj(9,:));
ylabel('$t_{\mathrm{engine}}$','Interpreter','latex','fontsize', 30);
%% Plot Controls
figure;
subplot(2,2,1);
stairs(ts(1:end-1),v_u(1,:));
ylabel('$j_{\textrm{brake}}$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
stairs(ts(1:end-1),v_u(2,:));
ylabel('$j_{\textrm{engine}}$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
stairs(ts(1:end-1),v_u(3,:));
ylabel('$\omega_{\textrm{steer}}$','Interpreter','latex','fontsize', 30);

%% Plot diagnostic internal state
% TODO: pass in the whole model for constants
% Slips
slip_f = atan2((v_traj(6,:)+0.5*v_traj(7,:)),v_traj(5,:))-v_traj(10,:);
slip_r = atan2((v_traj(6,:)+0.5*v_traj(7,:)),v_traj(5,:));
figure;
subplot(2,1,1);
plot(ts,slip_f);
ylabel('$\textrm{slip}_f$','Interpreter','latex','fontsize', 30);
subplot(2,1,2);
plot(ts,slip_r);
ylabel('$\textrm{slip}_r$','Interpreter','latex','fontsize', 30);
sgtitle('Slips','fontsize', 30);

% Forces
F_p_r = v_traj(9,:)-0.5*v_traj(8,:);
F_t_r = -5*slip_r*0.5;
F_p_f = -0.5*v_traj(8,:);
F_t_f = -5*slip_f*0.5;
figure;
subplot(2,2,1);
plot(ts,F_p_f);
ylabel('$F_{pf}$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
plot(ts,F_t_f);
ylabel('$F_{tf}$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
plot(ts,F_p_r);
ylabel('$F_{pr}$','Interpreter','latex','fontsize', 30);
subplot(2,2,4);
plot(ts,F_t_r);
ylabel('$F_{tr}$','Interpreter','latex','fontsize', 30);
sgtitle('Forces','fontsize', 30)

% force components
figure;
subplot(2,2,1);
plot(ts,-sin(v_traj(10,:)).*F_t_f + cos(v_traj(10,:)).*F_p_f);
ylabel('$F_{fu}$','Interpreter','latex','fontsize', 30);
yline(0,'--r');
subplot(2,2,2);
plot(ts,cos(v_traj(10,:)).*F_t_f + sin(v_traj(10,:)).*F_p_f);
ylabel('$F_{fv}$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
plot(ts,F_p_r);
ylabel('$F_{ru}$','Interpreter','latex','fontsize', 30);
subplot(2,2,4);
plot(ts,F_t_r);
ylabel('$F_{rv}$','Interpreter','latex','fontsize', 30);
sgtitle('Force Components','fontsize', 30)

%% Plot road trajectory
% Break out vehicle 
v_s = v_traj(1,:);
v_n = v_traj(2,:);
v_alpha = v_traj(3,:);
v_u = v_traj(5,:);
v_v = v_traj(6,:); 
v_delta = v_traj(10,:);

width = 3;
% Calculate left and right hand side
r_lx = r_x - width*sin(r_theta);
r_ly = r_y + width*cos(r_theta);
r_rx = r_x + width*sin(r_theta);
r_ry = r_y - width*cos(r_theta);

% Calculate vehicle poses
v_s(end) = 200; %hack to fix nans
v_theta = interp1(r_s,r_theta,v_s);
v_x = interp1(r_s,r_x,v_s) - v_n.*sin(v_theta);
v_y = interp1(r_s,r_y,v_s) + v_n.*cos(v_theta);

% Calculate velocities
rotation = exp((v_theta+v_alpha)*1i);
v_uv_rotated = rotation .* ([1 1i]*[v_u;v_v]);
v_u_rotated = real(v_uv_rotated);
v_v_rotated = imag(v_uv_rotated);

f=figure;
hold on;
plot(r_x,r_y, 'b');
plot(r_lx,r_ly, 'r');
plot(r_rx,r_ry, 'r');
plot(v_x,v_y,'g');
axis equal;
quiver(v_x,v_y,v_u_rotated,v_v_rotated);

end