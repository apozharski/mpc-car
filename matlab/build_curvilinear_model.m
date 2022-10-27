function model = build_curvilinear_model(varargin)
%build_car_model builds curvilenear model on top of existing model in
%vehicle frame.
%   Expects already generated model for vehicle dynamics in vehicle frame.
import casadi.*
if nargin == 1
    unfold_struct(varargin{1},'caller')
end
%% Constants
s_max = 5; % How long is the road

%% Variable declaration
s = MX.sym('s');                        % abscissa (distance along road).
n = MX.sym('n');                        % normal distance to road.
alpha = MX.sym('alpha');                % angle of vehicle to road.
%kappa = interpolant('kappa','bspline',{[0,2,4,6,8,10,15,20,22,25]},[0,.1,.6,.4,0,.2,-.4,.6,.4,.2]);    % deriv of orientation w.r.t s.
%kappa = Function('kappa',{s}, {.5*cos(pi*s)})
kappa = Function('kappa',{s}, {0});
W_l = Function('W_l', {s}, {MX(-1)});       % Width Left of the road center.
W_r = Function('W_r', {s}, {MX(1)});        % Width right of the road center.

dt = MX.sym('dt');

sym_x = [s;n;alpha;dt;x_veh];  % stack curvilinear model.
nx=length(sym_x);
sym_u = [u_veh];                      % controls equivalent in vehicle frame.

% Dynamics in curvilinear space (including clock state
s_prime = (u*cos(alpha)-v*sin(alpha))/(1-n*kappa(s));
n_prime = u*sin(alpha) + v*cos(alpha);
alpha_prime = omega - kappa(s)*s_prime;

% Explicit forward dynamics (with T_final as constant state).
expr_f_expl = dt*[s_prime;n_prime;alpha_prime;0;f_veh]

%% Build initial conditiona
% Define vehicle as starting at the center of the beginning of the road
% aligned directly along the intial orientation.
% T_final can range from 0 to infinity (practically it should never 
% actually be zero).
constr_Jbx_0 = blkdiag(eye(4),Jbx0_veh);
constr_lbx_0 = [0;0;0;0;x0_veh];
constr_ubx_0 = [0;0;0;2;x0_veh];

%% Build constraints
constr_Jbu = Jbu_veh;
constr_lbu = lbu_veh;
constr_ubu = ubu_veh;

% constr_Jbx = blkdiag([0,1,0,0;0,0,1,0],Jbx_veh);
% zeros(size(constr_Jbx,1),nx-size(constr_Jbx,2))
% constr_Jbx = [constr_Jbx,zeros(size(constr_Jbx,1),nx-size(constr_Jbx,2))];
% constr_lbx = [-1;-pi/2;lbx_veh];
% constr_ubx = [1;pi/2;ubx_veh];
constr_Jbx = blkdiag([1,0,0,0;0,1,0,0],Jbx_veh);
zeros(size(constr_Jbx,1),nx-size(constr_Jbx,2))
constr_Jbx = [constr_Jbx,zeros(size(constr_Jbx,1),nx-size(constr_Jbx,2))];
constr_lbx = [0;-1;lbx_veh];
constr_ubx = [s_max;1;ubx_veh];
% TODO use ACADOS h inequalities to model variable width.

%% Build terminal constraints

% Constrain us to be at the end of the track, with a reasonable angle to 
% the road and within the bounds. 
constr_Jbx_e = blkdiag([1,0,0,0;0,1,0,0],Jbx_e_veh);
zeros(size(constr_Jbx_e,1),nx-size(constr_Jbx_e,2));
constr_Jbx_e = [constr_Jbx_e,zeros(size(constr_Jbx_e,1),nx-size(constr_Jbx_e,2))];
constr_lbx_e = [s_max;-1;lbx_e_veh];
constr_ubx_e = [s_max;1;ubx_e_veh];

%% Build costs
% Terminal cost, TODO: Also energy, probably need to calculate that in the
%                      vehicle model (Architecture question).
% cost_expr_ext_cost = T_final*(.0001*u^2 + .0001*omega^2);
% cost_expr_ext_cost_e = T_final;
cost_expr_ext_cost = 500*dt + t_engine*t_brake + 0.001*j_engine^2+0.001*j_brake^2;%(s-s_max)^2 + u^2;
cost_expr_ext_cost_e = t_engine*t_brake;

%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end