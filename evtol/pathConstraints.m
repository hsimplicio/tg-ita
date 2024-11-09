function [c, ceq] = pathConstraints(time, state, control, params)
    % Evaluates path constraints for the EVTOL problem
    %
    % Inputs:
    %   x: state vector [x; y; vx; vy; E]
    %   u: control vector [Tx; Ty]
    %   p: parameter struct
    %
    % Outputs:
    %   c: inequality constraints
    %   ceq: equality constraints
    
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % No path constraints
    c = [];
    ceq = [];
end