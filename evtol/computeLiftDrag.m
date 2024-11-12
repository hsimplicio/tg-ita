function [lift, drag] = computeLiftDrag(velX, velY, params)
    % Computes aerodynamic forces for the EVTOL
    %
    % Inputs:
    %   velX: [1,n] = horizontal velocity component
    %   velY: [1,n] = vertical velocity component
    %   params: parameter struct
    %
    % Outputs:
    %   lift: [1,n] = lift force
    %   drag: [1,n] = drag force
    
    arguments
        velX double
        velY double
        params struct
    end
    
    % Extract parameters
    rho = params.environment.AIR_DENSITY;

    CDFus = params.aircraft.fuselage.CD;
    SfFus = params.aircraft.fuselage.Sf;
    StFus = params.aircraft.fuselage.St;
    
    SWing = params.aircraft.wing.S;
    eWing = params.aircraft.wing.e;
    CDpWing = params.aircraft.wing.CDp;
    ARWing = params.aircraft.wing.AR;
    CL0Wing = params.aircraft.wing.CL0;
    CLalphaWing = params.aircraft.wing.CLalpha;
    alphaFus = params.aircraft.wing.alphaFus;

    % Variables related to the wing lift coefficient
    M = params.aircraft.wing.sigmoid.M;
    alpha0 = params.aircraft.wing.sigmoid.alpha0;

    gamma = computeFlightAngle(velX, velY);
    alpha = -gamma;
    alphaWing = alpha + alphaFus; % Wing angle of attack
    
    % Compute Wing Lift Coefficient
    straightCL = CL0Wing + CLalphaWing * alphaWing;
    
    % Apply sigmoid function to merge linear and flat plate models
    sigMinus = exp(-M * (alphaWing - alpha0));
    sigPlus = exp(M * (alphaWing + alpha0));
    sigma = (1 + sigMinus + sigPlus) ./ ((1 + sigMinus) .* (1 + sigPlus));

    CD = CDpWing + straightCL .^ 2 / (pi * eWing * ARWing);
    CL = sigma .* (2 * sign(alphaWing) .* sin(alphaWing) .^ 2 .* cos(alphaWing)) + (1 - sigma) .* straightCL;
    
    V = sqrt(velX .^ 2 + velY .^ 2);

    LWing = 0.5 * rho * V .^ 2 * SWing .* CL;
    DWing = 0.5 * rho * V .^ 2 * SWing .* CD;

    % Fuselage drag
    DfFus = 0.5 * rho * velX .^ 2 * CDFus * SfFus;
    DtFus = 0.5 * rho * velY .^ 2 * CDFus * StFus;

    % Final computation
    lift = LWing;
    drag = sqrt(DfFus .^ 2 + DtFus .^ 2) + DWing;
end
