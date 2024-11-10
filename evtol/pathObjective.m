function L = pathObjective(time, state, control, params)
    % Evaluates running cost for the EVTOL problem
    %
    % Inputs:
    %   time: [1,n] = time vector
    %   state: [5,n] = state vector [x; y; vx; vy; E]
    %   control: [2,n] = control vector [Tx; Ty]
    %   params: parameter struct
    %
    % Outputs:
    %   L: [1,n] = instantaneous cost value
    
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % No running cost (only terminal cost)
    L = zeros(1, length(time));
end