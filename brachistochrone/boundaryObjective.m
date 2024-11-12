function cost = boundaryObjective(x0, xF, t0, tF, params)
    arguments
        x0 double
        xF double
        t0 double
        tF double
        params struct
    end
    
    % Minimize time
    cost = tF - t0;
end 