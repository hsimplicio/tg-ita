function cost = boundaryObjective(x0, xF, t0, tF, params)
    arguments
        x0 double
        xF double
        t0 double
        tF double
        params struct
    end
    
    % No boundary cost
    cost = 0;
end 