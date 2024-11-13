function dx = brachistochroneDynamics(time, state, control, params)
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % Extract states and control
    v = state(3,:);        % Velocity
    theta = control(1,:);  % Path angle
    g = params.GRAVITY;    % Gravity
    
    % System dynamics
    dx = [
        v .* cos(theta);           % dx/dt = v*cos(theta)
        v .* sin(theta);           % dy/dt = v*sin(theta)
        -g .* sin(theta);          % dv/dt = -g*sin(theta)
    ];
end 