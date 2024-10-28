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

    gamma = pi / 2 * ones(size(vx));

    condition_x = abs(vx) > 1e-4;
    condition_y = abs(vy) > 1e-4;

    gamma(condition_x) = atan2(vy(condition_x), vx(condition_x));

    gamma(~condition_x & condition_y) = sign(vy(~condition_x & condition_y)) * pi / 2;
end