function gamma = compute_gamma(vx, vy)
    % Compute gamma
    %
    % INPUTS:
    % vx = [1,n] = x velocity
    % vy = [1,n] = y velocity
    %
    % OUTPUTS:
    % gamma = [1,n] = flight path angle
    %

    % Compute gamma in the range [-pi, pi]
    gamma = atan2(vy, vx);

    % Correct gamma values to be within the range [-pi/2, pi/2]
    % and consider the case when vx ~ 0
    condition_1 = abs(gamma) > pi / 2;
    condition_2 = abs(vx) < 1e-4;

    % If gamma is outside the range [-pi/2, pi/2] and vx is not close to 0
    gamma(condition_1 & ~condition_2) = gamma(condition_1 & ~condition_2) - sign(gamma(condition_1 & ~condition_2)) * pi;

    % If gamma is outside the range [-pi/2, pi/2] and vx is close to 0
    gamma(condition_1 & condition_2) = sign(gamma(condition_1 & condition_2)) * pi / 2;
end