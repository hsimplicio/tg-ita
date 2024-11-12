function [c, ceq] = pathConstraints(time, state, control, params)
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % No path constraints needed
    c = [];   % Inequality constraints
    ceq = []; % Equality constraints
end 