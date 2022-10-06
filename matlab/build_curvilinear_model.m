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

sym_x = [s;n;alpha;x_veh;T_final];  % stack curvilinear vars on vehicle model.
sym_u = u_veh;                      % controls equivalent in vehicle frame.

% Dynamics in curvilinear space (including clock state
s_prime = (u*cos(alpha)-v*sin(alpha))/(1-n*kappa(s));
n_prime = u*sin(alpha) + v*cos(alpha);
alpha_prime = omega - kappa(s)*s_prime;

% Explicit forward dynamics (with T_final as constant state).
sym_xdot = T_final*[s_prime;n_prime;alpha_prime;f_veh;0];

%% Generic part
% (make of local workspace a struct and pass to output
names = who;
for ii = 1:length(names)
    eval([ 'model.' names{ii} '=' names{ii} ';'])
end
end