function [c, ceq] = pathConstraints(time, state, control, params)
    arguments
        time double
        state double
        control double
        params struct
    end

    g = params.GRAVITY;
    
    % Extract states
    sx = state(1,:);       % Horizontal position
    sy = state(2,:);       % Vertical position
    v = state(3,:);        % Velocity

    % Path constraints
    c = [];
    % Energy conservation (optional)
    E = 0.5 * v.^2 + g * sy;
    E0 = g * sy(1);  % Initial potential energy
    ceq = (E - E0)';  % Energy should be conserved

    % Debug output
    % disp('Energy conservation error:');
    % disp(['Max error: ', num2str(max(abs(ceq)))]);
    % disp(['Mean error: ', num2str(mean(abs(ceq)))]);
end 