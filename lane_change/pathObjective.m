function L = pathObjective(time, state, control, params)
    % Minimize acceleration and deviation from target velocity
    ax = control(1,:);
    ay = control(2,:);
    vx = state(3,:);
    
    % Weights for different objectives
    w_accel = 1.0;    % Weight for acceleration minimization
    w_speed = 0.1;    % Weight for speed tracking
    
    % Compute costs
    accel_cost = ax.^2 + ay.^2;  % Minimize acceleration (comfort)
    speed_cost = (vx - params.TARGET_VX).^2;  % Stay close to target speed
    
    L = w_accel * accel_cost + w_speed * speed_cost;
end 