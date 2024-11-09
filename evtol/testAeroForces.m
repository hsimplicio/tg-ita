function testAeroForces()
    % Test function to validate lift and drag calculations
    % across different angles of attack
    
    params = evtolParams();

    % Extract parameters
    AIR_DENSITY = params.environment.AIR_DENSITY;
    wingS = params.aircraft.wing.S;
    wingCL0 = params.aircraft.wing.CL0;
    wingCLalpha = params.aircraft.wing.CLalpha;
    wingAlphaFus = params.aircraft.wing.alphaFus;
    wingM = params.aircraft.wing.sigmoid.M;
    wingAlpha0 = params.aircraft.wing.sigmoid.alpha0;

    % Test velocities
    V = 25; % m/s
    alphaRange = (-30:1:30) * (pi/180); % Convert degrees to radians
    
    % Initialize arrays
    lift = zeros(size(alphaRange));
    drag = zeros(size(alphaRange));
    
    % Calculate forces for each angle of attack
    for i = 1:length(alphaRange)
        alpha = alphaRange(i);
        velX = V * cos(alpha);
        velY = V * sin(alpha);
        [lift(i), drag(i)] = computeLiftDrag(velX, velY, params);
    end
    
    % Calculate theoretical values for comparison
    % Linear region lift coefficient (before stall)
    linearCL = wingCL0 + wingCLalpha * (alphaRange + wingAlphaFus);
    linearLift = 0.5 * AIR_DENSITY * V^2 * wingS .* linearCL;
    
    % Plot results
    figure('Name', 'Aerodynamic Forces Validation');
    
    % Plot Lift
    subplot(2,2,1);
    plot(alphaRange*180/pi, lift, 'b-', 'LineWidth', 2, 'DisplayName', 'Computed Lift');
    hold on;
    plot(alphaRange*180/pi, linearLift, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Linear Theory');
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('Lift Force (N)');
    title('Lift vs Angle of Attack');
    legend('show');
    
    % Plot Drag
    subplot(2,2,2);
    plot(alphaRange*180/pi, drag, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('Drag Force (N)');
    title('Drag vs Angle of Attack');
    
    % Plot L/D ratio
    subplot(2,2,3);
    plot(alphaRange*180/pi, lift./drag, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('L/D Ratio');
    title('Lift-to-Drag Ratio');
    
    % Print key values
    [maxLD, maxLDIdx] = max(lift./drag);
    alphaMaxLD = alphaRange(maxLDIdx) * 180/pi;
    
    [maxL, maxLIdx] = max(lift);
    alphaMaxL = alphaRange(maxLIdx) * 180/pi;
    
    fprintf('\nKey Performance Parameters:\n');
    fprintf('Maximum L/D Ratio: %.2f at %.1f degrees\n', maxLD, alphaMaxLD);
    fprintf('Maximum Lift: %.2f N at %.1f degrees\n', maxL, alphaMaxL);
    fprintf('Stall Angle: %.1f degrees (theoretical)\n', wingAlpha0 * 180/pi);
    
    % Verify transition between linear and flat plate models
    subplot(2,2,4);
    straightCL = wingCL0 + wingCLalpha * (alphaRange + wingAlphaFus);
    M = wingM;
    alpha0 = wingAlpha0;
    
    sigMinus = exp(-M * (alphaRange - alpha0));
    sigPlus = exp(M * (alphaRange + alpha0));
    sigma = (1 + sigMinus + sigPlus) ./ ((1 + sigMinus) .* (1 + sigPlus));
    
    flatPlateCL = 2 * sign(alphaRange) .* sin(alphaRange).^2 .* cos(alphaRange);
    CLCombined = sigma .* flatPlateCL + (1 - sigma) .* straightCL;
    
    plot(alphaRange*180/pi, straightCL, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Linear Model');
    hold on;
    plot(alphaRange*180/pi, flatPlateCL, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Flat Plate');
    plot(alphaRange*180/pi, CLCombined, 'b-', 'LineWidth', 2, 'DisplayName', 'Combined Model');
    grid on;
    xlabel('Angle of Attack (degrees)');
    ylabel('Lift Coefficient');
    title('Lift Coefficient Models');
    legend('show');
end 