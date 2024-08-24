function derivatives = dae(path, states, controls, parameters, time, xad, iphase, workspace)
    % Define the DAEs
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
    derivatives = [
        vx; % xdot
        vy; % ydot
        (Tx - D * cos(gamma) - L * sin(gamma)) / m; % vxdot
        (Ty - D * sin(gamma) + L * cos(gamma) - m * g) / m; % vydot
        P_ind + P_forward; % Edot
    ];
end