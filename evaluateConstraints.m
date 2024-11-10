function [c, ceq] = evaluateConstraints(time, state, control, dynamics, defectConstraints, pathConstraints, boundaryConstraints)
    % Define the collocation constraints
    % Inputs:
    %   time: [1,n] = time vector
    %   state: [5,n] = state variables
    %   control: [2,n] = control variables
    %   dynamics: function = dynamics function
    %   defectConstraints: function = defect constraints function
    %   pathConstraints: function = path constraints function
    %   boundaryConstraints: function = boundary constraints function
    % Outputs:
    %   c = [m,1] = inequality constraints
    %   ceq = [m,1] = equality constraints


    % Initialize inequality and equality constraints
    c = [];
    ceq = [];

    % Evaluate defects constraints
    if ~isempty(defectConstraints)
        timeStep = (time(end) - time(1)) / (length(time) - 1);
        derivatives = dynamics(time, state, control);
        defects = defectConstraints(timeStep, state, derivatives);
        ceq = [ceq; defects(:)];
    end
    
    % Evaluate boundary constraints
    if ~isempty(boundaryConstraints)
        [boundaryIneq, boundaryEq] = boundaryConstraints(state(:,1), state(:,end), time(1), time(end));
        c = [c; boundaryIneq];
        ceq = [ceq; boundaryEq];
    end
    
    % Evaluate path constraints
    if ~isempty(pathConstraints)
        [pathIneq, pathEq] = pathConstraints(time, state, control);
        c = [c; pathIneq];
        ceq = [ceq; pathEq];
    end
end