function [z, packInfo] = packZ(tSpan, state, control, scaling)
    % Pack time bounds and trajectories into optimization vector
    % Inputs:
    %   tSpan: [1,2] = time span
    %   state: [nx,nGrid] = state trajectory
    %   control: [nu,nGrid] = control trajectory
    %   scaling: struct = scaling factors
    % Outputs:
    %   z: [nz,1] = decision variables
    %   packInfo: struct = information about the problem

    % Pack packInfo
    packInfo = struct();
    packInfo.nx = size(state,1);
    packInfo.nu = size(control,1);
    packInfo.nGrid = size(state,2);
    packInfo.scaling = scaling;

    % Scale variables
    tScaled = tSpan / scaling.timeScaling;
    stateScaled = state ./ repmat(scaling.stateScaling, 1, size(state,2));
    controlScaled = control ./ repmat(scaling.controlScaling, 1, size(control,2));

    % Pack time bounds and trajectories into optimization vector
    z = [
        tScaled(:);
        stateScaled(:);
        controlScaled(:)
    ];
end 