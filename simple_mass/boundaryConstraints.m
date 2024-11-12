function [c, ceq] = boundaryConstraints(x0, xF, t0, tF, boundaryConditions)
    arguments
        x0 double
        xF double
        t0 double
        tF double
        boundaryConditions struct
    end
    
    % Initial and final conditions
    ceq = [
        x0 - boundaryConditions.x0;  % Initial state
        xF - boundaryConditions.xF;  % Final state
    ];
    
    % No inequality constraints
    c = [];
end 