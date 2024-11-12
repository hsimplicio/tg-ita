function [time, state, control] = unpackZ(z, packInfo, scalingFlag)
    % Unpack optimization vector into components
    % Inputs:
    %   z: [nz,1] = decision variables
    %   packInfo: struct = information about the problem
    %   scalingFlag: logical = flag to indicate if scaling should be applied
    % Outputs:
    %   time: [1,nGrid] = time vector
    %   state: [nx,nGrid] = state variables
    %   control: [nu,nGrid] = control variables

    % Unpack packInfo
    nx = packInfo.nx;
    nu = packInfo.nu;
    nGrid = packInfo.nGrid;

    % Calculate expected sizes
    nStates = nx * nGrid;
    nControls = nu * nGrid;
    expectedLength = 2 + nStates + nControls;  % 2 time values + states + controls

    % Validate z length
    assert(length(z) == expectedLength, ...
        'z length mismatch. Expected %d elements but got %d', ...
        expectedLength, length(z));
    
    % Unpack optimization vector into components
    tSpan = z(1:2);
    z_rest = z(3:end);
    
    % Reshape state and control trajectories
    nStates = nx * nGrid;
    state = reshape(z_rest(1:nStates), [nx, nGrid]);
    control = reshape(z_rest(nStates+1:end), [nu, nGrid]);
    
    % Apply scaling if requested
    if nargin >= 3 && scalingFlag
        tSpan = tSpan * packInfo.scaling.timeScaling;
        state = state .* repmat(packInfo.scaling.stateScaling, 1, nGrid);
        control = control .* repmat(packInfo.scaling.controlScaling, 1, nGrid);
    end

    % Create time vector
    time = linspace(tSpan(1), tSpan(2), nGrid);
end 