function L = pathObjective(time, state, control, params)
    % Evaluates running cost (Lagrange term)
    % Inputs:
    %   time: [1,n] time vector
    %   state: [nx,n] state matrix
    %   control: [nu,n] control matrix
    %   params: parameter struct
    % Outputs:
    %   L: instantaneous cost value
    
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % TODO: Implement your path objective
    L = 0;
end 