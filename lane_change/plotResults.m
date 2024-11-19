function plotResults(figureName, z, saveFigures)
    if nargin < 3
        saveFigures = false;
    end
    
    figure('Name', figureName);
    
    % Vehicle trajectory
    subplot(3,1,1)
    plot(z.state(1,:), z.state(2,:), 'LineWidth', 2)
    xlabel('x [m]', 'FontSize', 12)
    ylabel('y [m]', 'FontSize', 12)
    title('Trajetória do Veículo', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    % axis equal
    
    % Velocities
    subplot(3,1,2)
    plot(z.time, z.state(3,:), 'LineWidth', 2, 'DisplayName', 'v_x')
    hold on
    plot(z.time, z.state(4,:), 'LineWidth', 2, 'DisplayName', 'v_y')
    xlabel('t [s]', 'FontSize', 12)
    ylabel('v [m/s]', 'FontSize', 12)
    title('Velocidades', 'FontSize', 12)
    legend('show', 'Location', 'best')
    grid on
    set(gca, 'FontSize', 12);
    
    % Accelerations
    subplot(3,1,3)
    plot(z.time, z.control(1,:), 'LineWidth', 2, 'DisplayName', 'a_x')
    hold on
    plot(z.time, z.control(2,:), 'LineWidth', 2, 'DisplayName', 'a_y')
    xlabel('t [s]', 'FontSize', 12)
    ylabel('a [m/s^2]', 'FontSize', 12)
    title('Entradas de Controle', 'FontSize', 12)
    legend('show', 'Location', 'best')
    grid on
    set(gca, 'FontSize', 12);
    
    if saveFigures
        mkdir('figures');
        saveas(gcf, ['figures/', figureName, '.fig']);
        saveas(gcf, ['figures/', figureName, '.svg']);
        saveas(gcf, ['figures/', figureName, '.png']);
    end
end 