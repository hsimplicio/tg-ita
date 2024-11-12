function dx = simpleMassDynamics(time, state, control, params)
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % Extract states and control
    v = state(2,:);        % Velocity
    F = control(1,:);      % Force
    m = params.MASS;       % Mass
    
    % System dynamics
    dx = [
        v;              % dx/dt = v
        F/m;           % dv/dt = F/m
    ];
end 