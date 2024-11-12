function L = pathObjective(time, state, control, params)
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % No running cost
    L = zeros(size(time));
end 