function cost = boundaryObjective(x0, xF, t0, tF, params)
    % Evaluates boundary cost (Mayer term)
    % Inputs:
    %   x0: [nx,1] initial state vector
    %   xF: [nx,1] final state vector
    %   t0: initial time
    %   tF: final time
    %   params: parameter struct
    % Outputs:
    %   cost: scalar cost value
    
    arguments
        x0 double
        xF double
        t0 double
        tF double
        params struct
    end
    
    % TODO: Implement your boundary objective
    cost = 0;
end 