function cost = objective(t, x, u, bnd_obj, path_obj)
    % Calculate the objective function value
    % Inputs:
    %   t: [1,n] = time vector
    %   x: [5,n] = state variables
    %   u: [2,n] = control variables
    %   bnd_obj: function = boundary objective function
    %   path_obj: function = path objective function
    %
    % Output:
    %   cost: scalar = objective function value
    %

    cost = 0;

    % Compute the cost integral along trajectory
    if ~isempty(path_obj)
        dt = (t(end) - t(1)) / (length(t) - 1);
        weights = [0.5, ones(1, length(t)-2), 0.5];
        integrands = path_obj(t,x,u);  % Calculate the integrands of the cost function
        integral_cost = dt * integrands .* weights;  % Trapezoidal integration
        cost = cost + sum(integral_cost);
    end

    % Compute the cost at the boundaries of the trajectory
    if ~isempty(bnd_obj)
        t0 = t(1);
        tF = t(end);
        x0 = x(:,1);
        xF = x(:,end);
        bnd_cost = bnd_obj(t0,x0,tF,xF);
        cost = cost + bnd_cost;
    end
end