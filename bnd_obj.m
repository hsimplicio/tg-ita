function cost = bnd_obj(x0, xF, t0, tF, p)
    % Function to compute boundary objective
    % Inputs:
    %   x0 = [5,1] = initial state
    %   xF = [5,1] = final state
    %   t0 = [1,1] = initial time
    %   tF = [1,1] = final time
    %   p = parameters struct
    % Output:
    %   cost = computed boundary objective value
    %
    arguments
        x0 double
        xF double
        t0 double
        tF double
        p struct
    end

    cost = xF(5); % final consumed energy
end