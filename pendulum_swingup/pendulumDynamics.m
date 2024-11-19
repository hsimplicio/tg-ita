function dx = pendulumDynamics(time, state, control, params)
    % Extract states and control
    theta = state(1,:);  % Angle
    omega = state(2,:);  % Angular velocity
    tau = control(1,:);  % Input torque
    
    % Extract parameters
    m = params.MASS;
    l = params.LENGTH;
    g = params.GRAVITY;
    b = params.DAMPING;
    
    % Compute derivatives
    dtheta = omega;
    domega = (tau - m*g*l*sin(theta) - b*omega) / (m*l^2);
    
    dx = [dtheta; domega];
end 