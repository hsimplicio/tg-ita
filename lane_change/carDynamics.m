function dx = carDynamics(time, state, control, params)
    % Simple double integrator dynamics
    % States: [x; y; vx; vy]
    % Controls: [ax; ay]
    
    % Extract velocities and accelerations
    vx = state(3,:);
    vy = state(4,:);
    ax = control(1,:);
    ay = control(2,:);
    
    % Simple kinematics
    dx = [
        vx;           % dx/dt = vx
        vy;           % dy/dt = vy
        ax;           % dvx/dt = ax
        ay            % dvy/dt = ay
    ];
end 