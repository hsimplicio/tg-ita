function [c, ceq] = path_cst(t, x, u)
    % Only include constraints that cannot be expressed as simple bounds
    %
    % Example of potential nonlinear constraints:
    % c = [
    %     sqrt(x(3,:).^2 + x(4,:).^2) - v_max;  % velocity magnitude constraint
    %     sqrt(u(1,:).^2 + u(2,:).^2) - T_max;  % total thrust constraint
    % ];
    %
    % Inputs:
    %   t: [1,n] = time vector
    %   x: [5,n] = state variables
    %   u: [2,n] = control variables
    %
    % Outputs:
    %   c: [m,1] = inequality constraints
    %   ceq: [m,1] = equality constraints
    %
    arguments
        t double
        x double
        u double
    end
    
    c = [];  % No additional inequality constraints needed
    ceq = [];  % No equality constraints needed
end