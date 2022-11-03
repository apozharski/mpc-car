function f = plot_solution(r_x,r_y,r_theta,r_s,v_traj,v_control,model,ts)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
close all;

%% Break out vehicle 
v_s = v_traj(1,:);
v_n = v_traj(2,:);
v_alpha = v_traj(3,:);
v_u = v_traj(5,:);
v_v = v_traj(6,:); 
v_omega = v_traj(7,:);
v_delta = v_traj(8,:);
w_fun = model.w_fun(r_s);
width = 3*w_fun.full();

dt = diff(ts);
%% Plot vehicle Curvilinear state
figure;
subplot(2,2,1);
plot(ts,v_s);
ylabel('$s$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
plot(ts,v_n);
ylabel('$n$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
plot(ts,v_alpha);
ylabel('$\alpha$','Interpreter','latex','fontsize', 30);

%% Plot vehicle CG state
kappa = model.kappa(v_s);
kappa = kappa.full();
figure;
subplot(2,2,1);
plot(ts,v_u);
ylabel('$u$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
plot(ts,v_v);
ylabel('$v$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
hold on;
plot(ts,v_omega);
ylabel('$\omega$','Interpreter','latex','fontsize', 30);
plot(ts,kappa,"--r");
hold off;
subplot(2,2,4);
plot(ts,v_delta);
ylabel('$\delta$','Interpreter','latex','fontsize', 30);

%% Plot Controls
figure;
subplot(2,2,1);
stairs(ts(1:end-1),model.k_brake*v_control(1,:));
ylabel('$t_{\textrm{brake}}$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
stairs(ts(1:end-1),model.k_engine*v_control(2,:));
ylabel('$t_{\textrm{engine}}$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
stairs(ts(1:end-1),v_control(3,:));
ylabel('$\omega_{\textrm{steer}}$','Interpreter','latex','fontsize', 30);

%% Plot diagnostic internal state
% TODO: pass in the whole model for constants
% Slips
slip_f = atan2((v_v+model.L_f*v_omega),v_u)-v_delta;
slip_r = atan2((v_v-model.L_r*v_omega),v_u);
figure;
subplot(2,1,1);
plot(ts,slip_f);
ylabel('$\textrm{slip}_f$','Interpreter','latex','fontsize', 30);
subplot(2,1,2);
plot(ts,slip_r);
ylabel('$\textrm{slip}_r$','Interpreter','latex','fontsize', 30);
sgtitle('Slips','fontsize', 30);

% Forces
F_p_r = model.k_engine*v_control(2,:)-model.k_brake*model.brake_bias*v_control(1,:);
F_t_r = -model.k_r*slip_r*(model.mass*model.weight_bias);
F_p_f = -model.k_brake*(1-model.brake_bias)*v_control(1,:);
F_t_f = -model.k_f*slip_f*(model.mass*(1-model.weight_bias));
figure;
subplot(2,2,1);
stairs(ts(1:end-1),F_p_f);
ylabel('$F_{pf}$','Interpreter','latex','fontsize', 30);
subplot(2,2,2);
plot(ts,F_t_f);
ylabel('$F_{tf}$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
stairs(ts(1:end-1),F_p_r);
ylabel('$F_{pr}$','Interpreter','latex','fontsize', 30);
subplot(2,2,4);
plot(ts,F_t_r);
ylabel('$F_{tr}$','Interpreter','latex','fontsize', 30);
sgtitle('Forces','fontsize', 30)

% force components
F_f_u = -sin(v_traj(8,1:end-1)).*F_t_f(1:end-1) + cos(v_traj(8,1:end-1)).*F_p_f;
F_f_v = cos(v_traj(8,1:end-1)).*F_t_f(1:end-1) + sin(v_traj(8,1:end-1)).*F_p_f;

figure;
subplot(2,2,1);
plot(ts(1:end-1),F_f_u);
ylabel('$F_{fu}$','Interpreter','latex','fontsize', 30);
yline(0,'--r');
subplot(2,2,2);
plot(ts(1:end-1),F_f_v);
ylabel('$F_{fv}$','Interpreter','latex','fontsize', 30);
subplot(2,2,3);
plot(ts(1:end-1),F_p_r);
ylabel('$F_{ru}$','Interpreter','latex','fontsize', 30);
subplot(2,2,4);
plot(ts(1:end-1),F_t_r(1:end-1));
ylabel('$F_{rv}$','Interpreter','latex','fontsize', 30);
sgtitle('Force Components','fontsize', 30);

% Lateral accel
figure;
plot(ts(1:end-1),dt.*(F_f_v + F_t_r(1:end-1))/model.mass);
ylabel('$\dot{v}$','Interpreter','latex','fontsize', 30);
%% Plot road trajectory

% Calculate left and right hand side
r_lx = r_x - width.*sin(r_theta);
r_ly = r_y + width.*cos(r_theta);
r_rx = r_x + width.*sin(r_theta);
r_ry = r_y - width.*cos(r_theta);

% Calculate vehicle poses
v_s(end) = model.s_max; %hack to fix nans
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
f.Position = [100 100 900 900];
movegui(f,'center');

%% make video for fun
% video = VideoWriter('video.avi');
% video.FrameRate = 1/dt;
% f = figure;
% open(video);
% for i=1:length(v_x)
%     
% end
end