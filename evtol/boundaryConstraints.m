function [c, ceq] = boundaryConstraints(x0, xF, t0, tF, boundaryConditions)
    % Evaluates boundary constraints for the EVTOL problem
    %
    % Inputs:
    %   x0: [5,1] = initial state vector [sx; sy; vx; vy; E]
    %   xF: [5,1] = final state vector [sx; sy; vx; vy; E]
    %   t0: double = initial time
    %   tF: double = final time
    %
    % Outputs:
    %   c: inequality constraints at boundaries
    %   ceq: equality constraints at boundaries
    
    arguments
        x0 double
        xF double
        t0 {mustBeNumeric}
        tF {mustBeNumeric}
        boundaryConditions struct
    end
    
    % Initial conditions
    initialPosX = x0(1);
    initialPosY = x0(2);
    initialVelX = x0(3);
    initialVelY = x0(4);
    initialEnergy = x0(5);
    
    % Final conditions
    finalPosX = xF(1);
    finalPosY = xF(2);
    finalVelX = xF(3);
    finalVelY = xF(4);
    
    
    % Equality constraints
    ceq = [
        % Initial conditions
        initialPosX - boundaryConditions.x0(1);      % x(0) = boundaryConditions.xLow(1)
        initialPosY - boundaryConditions.x0(2);      % y(0) = boundaryConditions.x0(2)
        initialVelX - boundaryConditions.x0(3);      % vx(0) = boundaryConditions.x0(3)
        initialVelY - boundaryConditions.x0(4);      % vy(0) = boundaryConditions.x0(4)
        initialEnergy - boundaryConditions.x0(5);  % E(0) = boundaryConditions.x0(5)
        
        % Final conditions
        finalPosX - boundaryConditions.xF(1);  % x(T) = boundaryConditions.xF(1)
        finalPosY - boundaryConditions.xF(2);  % y(T) = boundaryConditions.xF(2)
        finalVelX - boundaryConditions.xF(3);  % vx(T) = boundaryConditions.xF(3)
        finalVelY - boundaryConditions.xF(4);  % vy(T) = boundaryConditions.xF(4)
    ];
    
    % No inequality constraints
    c = [];
end