function dx = dyn_evtol(x, u, p)
    % Define the dynamics for the EVTOL problem
    %
    % INPUTS:
    %   x: [5,n] = [sx; sy; vx; vy; E] = state matrix
    %   u: [2,n] = [Tx; Ty] = control matrix
    %   p: parameter struct
    %
    % OUTPUTS:
    %   dx: [5,n] = [vx; vy; dvx; dvy; dE] = derivative of state matrix
    %
    arguments
        x double
        u double
        p struct
    end

    % Get the parameters
    g = p.environment.g;
    rho = p.environment.rho;
    m = p.aircraft.m;
    A = p.aircraft.A;
    chi = p.aircraft.chi;

    % Assign state variables
    sx = x(1,:);
    sy = x(2,:);
    vx = x(3,:);
    vy = x(4,:);
    E = x(5,:);
    
    % Assign control variables
    Tx = u(1,:);
    Ty = u(2,:);

    % Compute aerodynamic parameters
    V = sqrt(vx .^ 2 + vy .^ 2);
    gamma = compute_gamma(vx,vy);
    alpha_y = gamma;
    alpha_x = pi / 2 - gamma;

    % Compute the aerodynamic forces
    [L, D] = compute_lift_drag(vx,vy,p);

    % Compute the thrust forces
    T_rotor_x = Tx / 2;
    T_rotor_y = Ty / 4;

    % Induced velocity
    vh_x = sqrt(T_rotor_x / (2 * rho * A));
    vh_y = sqrt(T_rotor_y / (2 * rho * A));

    vi_x = induced_velocity(vh_x, V, alpha_x);
    vi_y = induced_velocity(vh_y, V, alpha_y);

    % Power requirements
    P_rotor_x = T_rotor_x .* vi_x;
    P_rotor_y = T_rotor_y .* vi_y;
    P_arm_x = 2.0 * P_rotor_x * (1 + chi);
    P_arm_y = 2.0 * P_rotor_y * (1 + chi);
    P_ind = 1.0 * P_arm_x + 2.0 * P_arm_y;

    P_forward = Tx .* V .* sin(alpha_x) + Ty .* V .* sin(alpha_y);

    % Compute the derivatives
    dx = [
        vx; % dx
        vy; % dy
        (Tx - D .* cos(gamma) - L .* sin(gamma)) / m; % vdx
        (Ty - D .* sin(gamma) + L .* cos(gamma) - m * g) / m; % vdy
        P_ind + P_forward; % dE
    ];
end