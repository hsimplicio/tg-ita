function plotResults(figureName, z, saveFigures)
    if nargin < 3
        saveFigures = false;
    end
    
    figure('Name', figureName);
    
    % Angle trajectory
    subplot(3,1,1)
    plot(z.time, z.state(1,:), 'b-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('\theta [rad]', 'FontSize', 12)
    title('Ângulo', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    % Angular velocity trajectory
    subplot(3,1,2)
    plot(z.time, z.state(2,:), 'r-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('\omega [rad/s]', 'FontSize', 12)
    title('Velocidade Angular', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    % Control input
    subplot(3,1,3)
    plot(z.time, z.control(1,:), 'g-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('\tau [N⋅m]', 'FontSize', 12)
    title('Torque', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    if saveFigures
        mkdir('figures');
        saveas(gcf, ['figures/', figureName, '.fig']);
        saveas(gcf, ['figures/', figureName, '.svg']);
        saveas(gcf, ['figures/', figureName, '.png']);
    end
end 