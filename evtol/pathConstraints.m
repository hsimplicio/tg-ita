function [c, ceq] = pathConstraints(time, state, control, params)
    % Evaluates path constraints for the EVTOL problem
    % Only include constraints that cannot be expressed as simple bounds
    %
    % Example of potential nonlinear constraints:
    % c = [
    %     sqrt(x(3,:).^2 + x(4,:).^2) - v_max;  % velocity magnitude constraint
    %     sqrt(u(1,:).^2 + u(2,:).^2) - T_max;  % total thrust constraint
    % ];
    %
    % Inputs:
    %   time: [1,n] = time vector
    %   state: [5,n] = state vector [sx; sy; vx; vy; E]
    %   control: [2,n] = control vector [Tx; Ty]
    %   params: parameter struct
    %
    % Outputs:
    %   c: [m,1] = inequality constraints
    %   ceq: [m,1] = equality constraints
    
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