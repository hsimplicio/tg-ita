function [c, ceq] = bnd_cst(x0, xF, t0, tF)
    % 
    % Inputs:
    %   x0: [5,1] = initial state
    %   xF: [5,1] = final state
    %   t0: double = initial time
    %   tF: double = final time
    %
    % Outputs:
    %   c = [m,1] = inequality constraints
    %   ceq = [m,1] = equality constraints
    %
    arguments
        x0 double
        xF double
        t0 double
        tF double
    end

    c = [];
    ceq = [
        x0(1);  % sx0 = 0
        x0(2);  % sy0 = 0
        x0(3);  % vx0 = 0
        x0(4);  % vy0 = 0
        x0(5);  % E0 = 0
        xF(1) - 1000;  % sxF = 1000
        xF(2) - 100;  % syF = 100
        xF(3) - 25;  % vxF = 25
        xF(4) - 0;  % vyF = 0
        t0;  % t0 = 0
        tF - 45;  % tF = 45
    ];

end