function model = build_curvilinear_model(varargin)
%build_car_model builds curvilenear model on top of existing model in
%vehicle frame.
%   Expects already generated model for vehicle dynamics in vehicle frame.
import casadi.*
if nargin == 1
    unfold_struct(varargin{1},'caller')
end
%% Variable declaration
s = MX.sym('s');                        % abscissa (distance along road).
n = MX.sym('n');                        % normal distance to road.
alpha = MX.sym('alpha');                % angle of vehicle to road.
s_prime = MX.sym('s_prime');
n_prime = MX.sym('n_prime');
alpha_prime = MX.sym('alpha_prime');
dt_prime = MX.sym('dt_prime');
s_end = MX.sym('s_end');

sym_x = [s;n;alpha;dt;x_veh];  % stack curvilinear model.
sym_xdot = [s_prime;n_prime;alpha_prime;dt_prime;x_dot_veh];
nx=length(sym_x);
sym_u = [u_veh];                      % controls equivalent in vehicle frame.
sym_p = [s_end];
%% Constants
s_finish = 300; % where is the finish line
kappa = interpolant('kappa','bspline', ...
    {[0,10,20,30,40,50,70,120,160,200,210,220,250,270,290]}, ...
    [0,.025,.05,0,-0.05,.01,-.025,-0.05,0.05,0,0.05,-0.05,-0.01,0.02,0.05]);    % deriv of orientation w.r.t s.
%kappa = interpolant('kappa','linear',{[0,20,30,125,130,170,180,285,290,300]},[0,pi/50,pi/100,pi/100,pi/50,0,-pi/100,-pi/100,-pi/50,0]);
%kappa = Function('kappa',{s}, {0.03*cos(0.01*pi*s)});
%kappa = Function('kappa',{s}, {0.003});
W_l = Function('W_l', {s}, {MX(-3)});       % Width Left of the road center.
W_r = Function('W_r', {s}, {MX(3)});        % Width right of the road center.
w_fun = Function('W_r', {s}, {MX(1)});
%w_fun = interpolant('w','linear',{[0,70,80,90,100,110,200]},[1,1,.6,.8,.6,1,1]);
%% Dynamics
% Dynamics in curvilinear space (including clock state
f_s_prime = (u*cos(alpha)-v*sin(alpha))/(1-n*kappa(s));
f_n_prime = u*sin(alpha) + v*cos(alpha);
f_alpha_prime = omega - kappa(s)*s_prime;

% Explicit forward dynamics (with T_final as constant state).
expr_f_impl = [s_prime - dt*f_s_prime
               n_prime - dt*f_n_prime
               alpha_prime - dt*f_alpha_prime
               dt_prime
               f_veh];

%% Build initial conditiona
% Define vehicle as starting at the center of the beginning of the road
% aligned directly along the intial orientation.
% T_final can range from 0 to infinity (practically it should never 
% actually be zero).
constr_Jbx_0 = blkdiag(eye(4),Jbx0_veh);
constr_lbx_0 = [0;0;0;0;x0_veh];
constr_ubx_0 = [0;0;0;3;x0_veh];

%% Build path constraints
constr_Jbu = Jbu_veh;
constr_lbu = lbu_veh;
constr_ubu = ubu_veh;

constr_Jbx = blkdiag([0,0,1,0],Jbx_veh);
zeros(size(constr_Jbx,1),nx-size(constr_Jbx,2));
constr_Jbx = [constr_Jbx,zeros(size(constr_Jbx,1),nx-size(constr_Jbx,2))];
constr_lbx = [-pi/2;lbx_veh];
constr_ubx = [pi/2;ubx_veh];
% TODO use ACADOS h inequalities to model variable width.
constr_expr_h = [n/w_fun(s)];
constr_lh =[-3];
constr_uh =[3];
%% Build terminal constraints

% Constrain us to be at the end of the track, with a reasonable angle to 
% the road and within the bounds. 
constr_Jbx_e = blkdiag([0,1,0,0],Jbx_e_veh);
zeros(size(constr_Jbx_e,1),nx-size(constr_Jbx_e,2));
constr_Jbx_e = [constr_Jbx_e,zeros(size(constr_Jbx_e,1),nx-size(constr_Jbx_e,2))];
constr_lbx_e = [-3,lbx_e_veh];
constr_ubx_e = [3,ubx_e_veh];
constr_expr_h_e = [s-s_end;n/w_fun(s)];
constr_lh_e =[0;-3];
constr_uh_e =[0;3];
%% Build costs
% Terminal cost, TODO: Also energy, probably need to calculate that in the
%                      vehicle model (Architecture question).
% cost_expr_ext_cost = T_final*(.0001*u^2 + .0001*omega^2);
% cost_expr_ext_cost_e = T_final;
cost_expr_ext_cost = 10*dt^2 + 1e-6*t_engine*t_brake + 1e-6*t_engine^2 + 1e-6*t_brake^2 + 1e-6*omega_steer^2;%(s-s_max)^2 + u^2;
cost_expr_ext_cost_e = 0;%t_engine*t_brake;

%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end