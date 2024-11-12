function compareWithAnalytical(z)
    % Compare numerical solution with analytical cycloid solution
    
    % Extract final conditions
    xf = z.state(1,end);
    yf = z.state(2,end);
    
    % Calculate cycloid parameters 

    % Calculate final angle
    f = @(theta_f) xf/yf - (theta_f - sin(theta_f))/(1 - cos(theta_f));
    theta_f = fsolve(f, pi/2, optimoptions('fsolve', 'Display', 'off'));

    % Calculate radius
    R = yf/(1 - cos(theta_f));
    
    % Generate analytical solution points
    theta = linspace(0, theta_f, 200);
    x_analytical = R*(theta - sin(theta));
    y_analytical = R*(1 - cos(theta));
    
    % Plot comparison
    figure('Name', 'Comparison with Analytical Solution');
    plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Numerical');
    hold on
    plot(x_analytical, y_analytical, 'r--', 'LineWidth', 2, 'DisplayName', 'Analytical');
    plot([0 xf], [0 yf], 'k.', 'MarkerSize', 20, 'DisplayName', 'Endpoints');
    grid on
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    title('Comparison with Analytical Solution (Cycloid)');
    legend('Location', 'best');
    axis equal
end 