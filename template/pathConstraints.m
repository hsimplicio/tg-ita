function [c, ceq] = pathConstraints(time, state, control, params)
    % Evaluates path constraints
    % Inputs:
    %   time: [1,n] time vector
    %   state: [nx,n] state matrix
    %   control: [nu,n] control matrix
    %   params: parameter struct
    % Outputs:
    %   c: inequality constraints
    %   ceq: equality constraints
    
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % TODO: Implement your path constraints
    c = [];
    ceq = [];
end 