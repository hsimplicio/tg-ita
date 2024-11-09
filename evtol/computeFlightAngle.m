function gamma = computeFlightAngle(velX, velY)
    % Computes flight path angle
    %
    % Inputs:
    %   velX: [1,n] = horizontal velocity component
    %   velY: [1,n] = vertical velocity component
    %
    % Outputs:
    %   gamma: [1,n] = flight path angle [rad]
    
    arguments
        velX double
        velY double
    end
    
    gamma = pi / 2 * ones(size(velX));

    conditionX = abs(velX) > 1e-4;
    conditionY = abs(velY) > 1e-4;

    gamma(conditionX) = atan2(velY(conditionX), velX(conditionX));

    gamma(~conditionX & conditionY) = sign(velY(~conditionX & conditionY)) * pi / 2;
end