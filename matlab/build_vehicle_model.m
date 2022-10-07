function model = build_vehicle_model()
% build_vehicle_model creates a simple single track (bycicle) model with
% the center of gravity being in the middle of the object.
    model = struct;

    %% constants
    L   = 1;
    l_r = 0.5;

    %% states
    delta = 0;

    %% vehicle model ODEs
    beta = atan(l_r * tan(delta) / L);
    x_dot = velocity * cos(beta + theta);
    y_dot = velocity * sin(beta + theta);
    theta_dot = velocity * tan(delta) * cos(beta) / L;
    delta_dot = phi; % steering rate
end

