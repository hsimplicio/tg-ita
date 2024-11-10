function J = evaluateObjective(time, state, control, boundaryObjective, pathObjective)
    % Evaluate the combined objective function
    % Inputs:
    %   time: [1,n] = time vector
    %   state: [nx,n] = state variables
    %   control: [nu,n] = control variables
    %   boundaryObjective: function = Mayer term (φ)
    %   pathObjective: function = Lagrange term (L)
    %
    % Outputs:
    %   J = scalar = objective value
    
    % Initialize objective
    J = 0;
    
    % Add Mayer term if provided
    if ~isempty(boundaryObjective)
        J = J + boundaryObjective(state(:,1), state(:,end), time(1), time(end));
    end
    
    % Add Lagrange term if provided
    if ~isempty(pathObjective)
        % Trapezoidal integration of the path objective
        integrand = pathObjective(time, state, control);
        J = J + trapz(time, integrand);
    end
end 