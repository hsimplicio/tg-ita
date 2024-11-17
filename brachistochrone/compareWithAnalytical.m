function compareWithAnalytical(z, plotFlag)
    % Compare numerical solution with analytical cycloid solution

    if nargin < 2
        plotFlag = false;
    end
    
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
    plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Numérica');
    hold on
    plot(x_analytical, y_analytical, 'r--', 'LineWidth', 2, 'DisplayName', 'Analítica');
    plot([0 xf], [0 yf], 'k.', 'MarkerSize', 20, 'DisplayName', 'Pontos Iniciais e Finais');
    grid on
    xlabel('x [m]', 'FontSize', 12);
    ylabel('y [m]', 'FontSize', 12);
    title('Comparação com Solução Analítica (Ciclóide)', 'FontSize', 12);
    legend('Location', 'best', 'FontSize', 12);
    axis equal
    set(gca, 'FontSize', 12);

    if plotFlag
        mkdir('figures');
        savefig('figures/Brachistochrone Comparison.fig');
        saveas(gcf, 'figures/Brachistochrone Comparison.svg');
        saveas(gcf, 'figures/Brachistochrone Comparison.png');
    end
end 