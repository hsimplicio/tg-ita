function L = pathObjective(time, state, control, params)
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % Minimize control effort
    L = control(1,:).^2;
end 