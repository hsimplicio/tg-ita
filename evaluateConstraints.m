function [c, ceq] = evaluateConstraints(z, packInfo, dynamics, defectConstraints, pathConstraints, boundaryConstraints)
    % Define the collocation constraints
    % Inputs:
    %   z: [nz,1] = decision variables
    %   packInfo: struct = information about the problem
    %   dynamics: function = dynamics function
    %   defectConstraints: function = defect constraints function
    %   pathConstraints: function = path constraints function
    %   boundaryConstraints: function = boundary constraints function
    % Outputs:
    %   c = [m,1] = inequality constraints
    %   ceq = [m,1] = equality constraints

    % Check for NaN values in z
    if any(isnan(z))
        error('NaN values detected in decision variables');
    end
    
    % Unpack z
    [time, state, control] = unpackZ(z, packInfo);
    [physicalTime, physicalState, physicalControl] = unpackZ(z, packInfo, true);

    % Initialize inequality and equality constraints
    c = [
        time(1) - time(end)  % t0 <= tf
    ];
    ceq = [];

    % Evaluate defects constraints
    if ~isempty(defectConstraints)
        % Evaluate derivatives
        derivatives = dynamics(physicalTime, physicalState, physicalControl);
        % Scale derivatives
        derivatives = derivatives ./ packInfo.scaling.stateScaling * packInfo.scaling.timeScaling;
        % Evaluate defects
        timeStep = (time(end) - time(1)) / packInfo.nGrid;
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