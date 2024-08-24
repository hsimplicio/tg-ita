function [L, D] = lift_drag(V, vx, vy, alpha, acFx, acFh, airdensity, acFusCD, acWingS, acWing_e, acWingCD_p, acWingAR, acWingCL_zero, acWingCL_alpha)
    % Usage: [L, D] = lift_drag(V, vx, vy, alpha, acFx, acFh, airdensity, acFusCD, acWingS, acWing_e, acWingCD_p, acWingAR, acWingCL_zero, acWingCL_alpha);
    arguments
        V double
        vx double
        vy double
        alpha double
        acFx double
        acFh double
        airdensity double
        acFusCD double
        acWingS double
        acWing_e double
        acWingCD_p double
        acWingAR double
        acWingCL_zero double
        acWingCL_alpha double
    end

    % Variables related to the wing lift coefficient
    M = 4.0;
    alpha_0 = (pi / 180) * 20;
    sgn_alpha = 0; % Initialization
    alpha_wing = alpha + 0 * pi / 180; % Wing angle of attack

    % Compute Wing Lift Coefficient
    straight_CL = acWingCL_zero + acWingCL_alpha * alpha_wing;

    % Set sign of alpha
    if alpha_wing > 0
        sgn_alpha = 1;
    elseif alpha_wing < 0
        sgn_alpha = -1;
    end

    % Sigmoide function
    sig_minus = exp(-M * (alpha_wing - alpha_0));
    sig_plus = exp(M * (alpha_wing + alpha_0));
    sigma = (1 + sig_minus + sig_plus) / ((1 + sig_minus) * (1 + sig_plus));

    CD = acWingCD_p + straight_CL^2 / (pi * acWing_e * acWingAR);
    CL = sigma * (2 * sgn_alpha * sin(alpha_wing)^2 * cos(alpha_wing)) + (1 - sigma) * straight_CL;
    wingLift = 0.5 * airdensity * V^2 * acWingS * CL;
    wingDrag = 0.5 * airdensity * V^2 * acWingS * CD;

    % Fuselage drag
    Dx = 0.5 * airdensity * vx * vx * acFusCD * acFx;
    Dh = 0.5 * airdensity * vy * vy * acFusCD * acFh;

    % Final computation
    L = wingLift;
    D = sqrt(Dx^2 + Dh^2) + wingDrag;
end