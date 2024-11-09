function L = pathObjective(time, state, control, params)
    % Evaluates running cost for the EVTOL problem
    %
    % Inputs:
    %   time: time vector
    %   state: state vector [x; y; vx; vy; E]
    %   control: control vector [Tx; Ty]
    %   params: parameter struct
    %
    % Outputs:
    %   L: instantaneous cost value
    
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % No running cost (only terminal cost)
    L = 0;
end