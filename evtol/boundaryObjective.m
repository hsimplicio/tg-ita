function cost = boundaryObjective(x0, xF, t0, tF, p)
    % Evaluates boundary cost for the EVTOL problem
    %
    % Inputs:
    %   x0: [5,1] = initial state vector [sx; sy; vx; vy; E]
    %   xF: [5,1] = final state vector [sx; sy; vx; vy; E]
    %   t0: double = initial time
    %   tF: double = final time
    %   p: parameter struct
    %
    % Outputs:
    %   cost: scalar cost value
    
    arguments
        x0 double
        xF double
        t0 double
        tF double
        p struct
    end
    
    % Extract final energy from state
    finalEnergy = xF(5);
    
    % Minimize final energy
    cost = finalEnergy;
end