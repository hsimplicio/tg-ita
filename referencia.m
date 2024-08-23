%% Find the induced velocity
function resultado = vi_fun(x, vh, V, alpha)
    arguments
        x double
        vh double
        V double
        alpha double
    end

    % replace this with your equation
    resultado = x - vh * vh / (sqrt((V * cos(alpha)) * (V * cos(alpha)) + (V * sin(alpha) + x) * (V * sin(alpha) + x)));
end

%% 
function resultado = fzero(a,b,vh,V,alpha)
    arguments
        a double
        b double
        vh double
        V double
        alpha double
    end

    tol = 1e-6;
    fa = viFun(a, vh, V, alpha);
    fb = viFun(b, vh, V, alpha);
    if fa * fb >= 0
        resultado = NaN;
        return;
    end 
    
    while (b - a) > tol
        c = (a + b) / 2;
        fc = viFun(c, vh, V, alpha);
        if fc == 0
            resultado = c;
            return;
        elseif fa * fc < 0
            b = c;
            % fb = fc;
        else
            a = c;
            fa = fc;
        end
    end

    resultado = (a + b) / 2;
end

%% Aerodynamic Parameters Computation
%% Aerodynamic Forces Computation
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

%% Define the end point (Mayer) cost function
function total_energy = endpoint_cost(initial_states, final_states, parameters, t0, tf, xad, iphase, workspace)
    arguments
        initial_states adouble
        final_states adouble
        parameters adouble
        t0 adouble
        tf adouble
        xad adouble
        iphase int
        workspace Workspace
    end

    % Define the end point cost function
    total_energy = final_states(5);
end
% Instantiate an object of the adouble class
% ad = adouble();

%% Define the integrand (Lagrange) cost function
function integrand_cost = integrand_cost(states, controls, parameters, time, iphase, workspace)
    arguments
        states adouble
        controls adouble
        parameters adouble
        time adouble
        iphase int
        workspace Workspace
    end

    % Define the integrand cost function
    integrand_cost = 0.0;
end

%% Define the DAEs
function derivatives = dae(path, states, controls, parameters, time, xad, iphase, workspace)
    arguments
        path adouble
        states adouble
        controls adouble
        parameters adouble
        time adouble
        xad adouble
        iphase int
        workspace Workspace
    end

    % Define some parameters
    m = 240.0;
    g = 9.78;
    air_density = 1.225;
    A = 1.6;
    ac_wing_s = 4.0;
    ac_wing_e = 0.9;
    ac_wing_ar = 20.0;
    chi = 1.0;
    % aerodynamic parameters
    ac_fx = 2.11;
    ac_fh = 1.47;
    ac_fus_cd = 1.0;
    ac_wing_cd_p = 0.0437;
    ac_wing_cl_zero = 0.28;
    ac_wing_cl_alpha = 4.00;
    x = states(1);
    y = states(2);
    vx = states(3);
    vy = states(4);
    energy = states(5);
    Tx = controls(1);
    Ty = controls(2);

    % aerodynamica computed parameters: velocity and gamma
    V = sqrt(vx.value()^2 + vy.value()^2);
    gamma = pi / 2;

    if abs(vx.value()) > 1e-4
        gamma = atan(vy.value() / vx.value());
    elseif abs(vy.value()) > 1e-4
        if vy.value() > 0
            gamma = pi / 2;
        else
            gamma = -pi / 2;
        end
    end

    % Compute the aerodynamic forces
    [L, D] = lift_drag(V, vx.value(), vy.value(), -gamma, ac_fx, ac_fh, air_density, ac_fus_cd, ac_wing_s, ac_wing_e, ac_wing_cd_p, ac_wing_ar, ac_wing_cl_zero, ac_wing_cl_alpha);

    % Compute the thrust forces
    T_rotor_x = Tx / 2;
    T_rotor_y = Ty / 4;

    % Induced speed
    vh_x = sqrt(T_rotor_x / (2 * air_density * A));
    vh_y = sqrt(T_rotor_y / (2 * air_density * A));
    vi_x = fzero(-50, 50, vh_x, V, pi / 2 - gamma);
    vi_y = fzero(-50, 50, vh_y, V, gamma);

    % Power requirements
    P_rotor_x = T_rotor_x * vi_x;
    P_rotor_y = T_rotor_y * vi_y;
    P_arm_x = 2.0 * P_rotor_x * (1 + chi);
    P_arm_y = 2.0 * P_rotor_y * (1 + chi);
    P_ind = 1.0 * P_arm_x + 2.0 * P_arm_y;
    P_forward = 2.0 * T_rotor_x * V * sin(pi / 2 - gamma) + 4.0 * T_rotor_y * V * sin(gamma);

    % Compute the derivatives
    derivatives = zeros(5, 1); % (xdot, ydot, vxdot, vydot, Edot)
    derivatives(1) = vx;
    derivatives(2) = vy;
    derivatives(3) = (Tx - D * cos(gamma) - L * sin(gamma)) / m;
    derivatives(4) = (Ty - D * sin(gamma) + L * cos(gamma) - m * g) / m;
    derivatives(5) = P_ind + P_forward;
end