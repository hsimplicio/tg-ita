function [c, ceq] = boundaryConstraints(x0, xF, t0, tF, boundaryConditions)
    % Evaluates boundary constraints
    % Inputs:
    %   x0: [nx,1] initial state vector
    %   xF: [nx,1] final state vector
    %   t0: initial time
    %   tF: final time
    %   boundaryConditions: struct with boundary conditions
    % Outputs:
    %   c: inequality constraints at boundaries
    %   ceq: equality constraints at boundaries
    
    arguments
        x0 double
        xF double
        t0 double
        tF double
        boundaryConditions struct
    end
    
    % TODO: Implement your boundary constraints
    c = [];
    ceq = [];
end 