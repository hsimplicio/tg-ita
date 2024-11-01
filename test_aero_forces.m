function test_aero_forces()
    % Test function to validate lift and drag calculations
    % across different angles of attack
    
    % Setup test parameters
    p.environment.g = 9.78;
    p.environment.rho = 1.2;
    p.aircraft.m = 240.0;
    p.aircraft.A = 1.6;
    p.aircraft.chi = 1.0;
    p.aircraft.fuselage.CD = 1.0;
    p.aircraft.fuselage.Sf = 2.11;
    p.aircraft.fuselage.St = 1.47;
    p.aircraft.wing.S = 4.0;
    p.aircraft.wing.e = 0.9; 
    p.aircraft.wing.CD_p = 0.0437;
    p.aircraft.wing.AR = 20.0;
    p.aircraft.wing.CL_zero = 0.28;
    p.aircraft.wing.CL_alpha = 4.00;
    p.aircraft.wing.alpha_fus = 0 * (pi / 180);
    p.aircraft.wing.sigmoid.M = 4.0;
    p.aircraft.wing.sigmoid.alpha_0 = 20 * (pi / 180);

    % Test velocities
    V = 25; % m/s
    alpha_range = (-30:1:30) * (pi/180); % Convert degrees to radians
    
    % Initialize arrays
    L = zeros(size(alpha_range));
    D = zeros(size(alpha_range));
    
    % Calculate forces for each angle of attack
    for i = 1:length(alpha_range)
        alpha = alpha_range(i);
        vx = V * cos(alpha);
        vy = V * sin(alpha);
        [L(i), D(i)] = compute_lift_drag(vx, vy, p);
    end
    
    % Calculate theoretical values for comparison
    % Linear region lift coefficient (before stall)
    CL_linear = p.aircraft.wing.CL_zero + p.aircraft.wing.CL_alpha * (alpha_range + p.aircraft.wing.alpha_fus);
    L_linear = 0.5 * p.environment.rho * V^2 * p.aircraft.wing.S .* CL_linear;
    
    % Plot results
    figure('Name', 'Aerodynamic Forces Validation');
    
    % Plot Lift
    subplot(2,2,1);
    plot(alpha_range*180/pi, L, 'b-', 'LineWidth', 2, 'DisplayName', 'Computed Lift');
    hold on;
    plot(alpha_range*180/pi, L_linear, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Linear Theory');
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('Lift Force (N)');
    title('Lift vs Angle of Attack');
    legend('show');
    
    % Plot Drag
    subplot(2,2,2);
    plot(alpha_range*180/pi, D, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('Drag Force (N)');
    title('Drag vs Angle of Attack');
    
    % Plot L/D ratio
    subplot(2,2,3);
    plot(alpha_range*180/pi, L./D, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('L/D Ratio');
    title('Lift-to-Drag Ratio');
    
    % Print key values
    [max_LD, max_LD_idx] = max(L./D);
    alpha_max_LD = alpha_range(max_LD_idx) * 180/pi;
    
    [max_L, max_L_idx] = max(L);
    alpha_max_L = alpha_range(max_L_idx) * 180/pi;
    
    fprintf('\nKey Performance Parameters:\n');
    fprintf('Maximum L/D Ratio: %.2f at %.1f degrees\n', max_LD, alpha_max_LD);
    fprintf('Maximum Lift: %.2f N at %.1f degrees\n', max_L, alpha_max_L);
    fprintf('Stall Angle: %.1f degrees (theoretical)\n', p.aircraft.wing.sigmoid.alpha_0 * 180/pi);
    
    % Verify transition between linear and flat plate models
    subplot(2,2,4);
    straight_CL = p.aircraft.wing.CL_zero + p.aircraft.wing.CL_alpha * (alpha_range + p.aircraft.wing.alpha_fus);
    M = p.aircraft.wing.sigmoid.M;
    alpha_0 = p.aircraft.wing.sigmoid.alpha_0;
    
    sig_minus = exp(-M * (alpha_range - alpha_0));
    sig_plus = exp(M * (alpha_range + alpha_0));
    sigma = (1 + sig_minus + sig_plus) ./ ((1 + sig_minus) .* (1 + sig_plus));
    
    flat_plate_CL = 2 * sign(alpha_range) .* sin(alpha_range).^2 .* cos(alpha_range);
    CL_combined = sigma .* flat_plate_CL + (1 - sigma) .* straight_CL;
    
    plot(alpha_range*180/pi, straight_CL, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Linear Model');
    hold on;
    plot(alpha_range*180/pi, flat_plate_CL, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Flat Plate');
    plot(alpha_range*180/pi, CL_combined, 'b-', 'LineWidth', 2, 'DisplayName', 'Combined Model');
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('Lift Coefficient');
    title('Lift Coefficient Models');
    legend('show');
end 