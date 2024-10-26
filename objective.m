function cost = objective(t, x, u, path_obj, bnd_obj)
    % Calculate the objective function value
    % Inputs:
    %   t = [1,n] = time vector
    %   x = [5,n] = state variables
    %   u = [2,n] = control variables
    %   path_obj = [1,n] = path objective function
    %   bnd_obj = [1,n] = boundary objective function
    %
    % Output:
    %   cost = [1,n] = objective function value
    %

    cost = zeros(1,len(t));

    % Compute the cost integral along trajectory
    if ~isempty(path_obj)
        dt = (t(end)-t(1))/(len(t)-1);
        integrand = path_obj(t,x,u);  % Calculate the integrand of the cost function
        integral_cost = dt*integrand;  % Trapezoidal integration
        cost = cost + integral_cost;
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