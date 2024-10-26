function [L, D] = compute_lift_drag(vx, vy, p)
    % Compute the lift and drag forces
    %
    % INPUTS:
    % vx = [1,n] = x velocity
    % vy = [1,n] = y velocity
    % p = parameter struct
    %
    % OUTPUTS:
    % L = [1,n] = lift force
    % D = [1,n] = drag force
    %
    arguments (Input)
        vx double
        vy double
        p struct
    end
    
    arguments (Output)
        L double
        D double
    end
    
    rho = p.environment.rho;

    CD_fus = p.aircraft.fuselage.CD;
    Sf_fus = p.aircraft.fuselage.Sf;
    St_fus = p.aircraft.fuselage.St;
    
    S_wing = p.aircraft.wing.S;
    e_wing = p.aircraft.wing.e;
    CD_p_wing = p.aircraft.wing.CD_p;
    AR_wing = p.aircraft.wing.AR;
    CL_zero_wing = p.aircraft.wing.CL_zero;
    CL_alpha_wing = p.aircraft.wing.CL_alpha;
    alpha_fus = p.aircraft.wing.alpha_fus;

    % Variables related to the wing lift coefficient
    M = p.aircraft.wing.sigmoid.M;
    alpha_0 = p.aircraft.wing.sigmoid.alpha_0;

    alpha = -compute_gamma(vx, vy);  %
    alpha_wing = alpha + alpha_fus; % Wing angle of attack
    
    % Compute Wing Lift Coefficient
    straight_CL = CL_zero_wing + CL_alpha_wing * alpha_wing;
    
    % Apply sigmoid function to merge linear and flat plate models
    sigma = sigmoid(M * (alpha_wing - alpha_0)) + sigmoid(-M * (alpha_wing + alpha_0));
    
    CD = CD_p_wing + straight_CL .^ 2 / (pi * e_wing * AR_wing);
    CL = sigma .* (2 * sign(alpha_wing) .* sin(alpha_wing) .^ 2 .* cos(alpha_wing)) + (1 - sigma) .* straight_CL;
    
    V = sqrt(vx .^ 2 + vy .^ 2);

    L_wing = 0.5 * rho * V .^ 2 * S_wing .* CL;
    D_wing = 0.5 * rho * V .^ 2 * S_wing .* CD;

    % Fuselage drag
    Df_fus = 0.5 * rho * vx .^ 2 * CD_fus * Sf_fus;
    Dt_fus = 0.5 * rho * vy .^ 2 * CD_fus * St_fus;

    % Final computation
    L = L_wing;
    D = sqrt(Df_fus .^ 2 + Dt_fus .^ 2) + D_wing;
end