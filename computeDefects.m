function defects = computeDefects(timeStep, state, stateDerivatives)
    % This function computes the defects for
    % direct transcription using the Trapezoidal Rule.
    %
    % Inputs:
    %   timeStep: scalar = time step
    %   state: [nStates, nGrid] = matrix of states
    %   stateDerivatives: [nStates, nGrid-1] = matrix of state derivatives
    %
    % Outputs:
    %   defects: [nStates, nGrid-1] = matrix of defects


    nGrid = size(state, 2);

    indexLower = 1:(nGrid-1);
    indexUpper = 2:nGrid;

    stateLower = state(:,indexLower);
    stateUpper = state(:,indexUpper);

    derivativesLower = stateDerivatives(:,indexLower);
    derivativesUpper = stateDerivatives(:,indexUpper);

    % Apply the Trapezoidal Rule
    defects = stateUpper - stateLower - 0.5 * timeStep * (derivativesLower + derivativesUpper);
end