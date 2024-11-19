function L = pathObjective(time, state, control, params)
    % Minimize control effort (quadratic cost)
    L = control(1,:).^2;
end 