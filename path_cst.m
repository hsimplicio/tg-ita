function [c, ceq] = path_cst(t, x, u)
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

    c = [
        (x(1,:) - 1000)';  % sx <= 1000
        (x(2,:) - 110)';  % sy <= 110
        (x(3,:) - 35)';  % vx <= 35
        (x(4,:) - 6)';  % vy <= 6
        (x(1,:) - 0)';  % sx >= 0
        (x(2,:) - -10)';  % sy >= -10
        (x(3,:) - 0)';  % vx >= 0
        (x(4,:) - -5)';  % vy >= -5
        (x(5,:) - 0)';  % E >= 0
        (u(1,:) - 1800)';  % Fx <= 1800
        (u(2,:) - 2600)';  % Fy <= 2600
        (u(1,:) - 0)';  % Fx >= 0
        (u(2,:) - 0)';  % Fy >= 0
    ];
    ceq = [];
end