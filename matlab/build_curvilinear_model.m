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
T_final = MX.sym('T');                  % Final time Parameter
kappa = Function('kappa', {s}, {0});    % deriv of orientation w.r.t s.
W_l = Function('W_l', {s}, (-1));       % Width Left of the road center.
W_r = Function('W_r', {s}, {1});        % Width right of the road center.


sym_x = [s;n;alpha;x_veh;T_final];  % stack curviliner model.
sym_u = u_veh;                      % controls equivalent in vehicle frame.

% Dynamics in curvilinear space (including clock state
s_prime = (u*cos(alpha)-v*sin(alpha))/(1-n*kappa(s));
n_prime = u*sin(alpha) + v*cos(alpha);
alpha_prime = omega - kappa(s)*s_prime;

% Explicit forward dynamics (with T_final as constant state).
sym_xdot = T_final*[s_prime;n_prime;alpha_prime;f_veh;0];

%% Build costs
% Terminal cost, TODO: Also energy, probably need to calculate that in the
%                      vehicle model (Architecture question).

cost_expr_ext_cost_e = T_final;

%% Build initial conditiona
% Define vehicle as starting at the center of the beginning of the road
% aligned directly along the intial orientation.
% T_final can range from 0 to infinity (practically it should never 
% actually be zero).
constr_lbx_0 = [0;0;0;x0_veh;0];
constr_ubx_0 = [0;0;0;x0_veh;inf];

%% Build constraints
constr_lbu = lbu_veh;
constr_ubu = ubu_veh;

constr_lbx = [-inf;-1;-pi/2;lbx_veh;0];
constr_ubx = [-inf;1;pi/2;ubx_veh;inf];

% TODO use ACADOS h inequalities to model variable width.

%% Build terminal constraints
s_max = 100; % How long is the road

% Constrain us to be at the end of the track, with a reasonable angle to 
% the road and within the bounds. 
% TODO: implement h_e for variable road width.
constr_lbx_e = [s_max;-1;-pi/2;lbx_e_veh;0];
constr_ubx_e = [s_max;1;pi/2;ubx_e_veh;inf];

%% Generic part
% (make local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end