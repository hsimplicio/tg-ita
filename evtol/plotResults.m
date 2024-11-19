function plotResults(figureName, z, saveFigures)
    % Plot the results of the EVTOL trajectory
    % figureName: name of the figure
    % z: solution structure
    % saveFigures: boolean to save the figures

    if nargin < 3
        saveFigures = false;
    end

    figure('Name', figureName);
    
    % Trajectory plot
    subplot(2,2,1)
    plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2)
    xlabel('x [m]', 'FontSize', 12)
    ylabel('y [m]', 'FontSize', 12)
    title('Trajetória do Vôo', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    % Velocity plot
    subplot(2,2,2)
    plot(z.time, z.state(3,:), 'r-', ...
         z.time, z.state(4,:), 'b-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('v [m/s]', 'FontSize', 12)
    legend('v_x', 'v_y', 'FontSize', 12)
    title('Velocidades', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    % Energy plot
    subplot(2,2,3)
    plot(z.time, z.state(5,:), 'k-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('E [J]', 'FontSize', 12)
    title('Consumo de Energia', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    % Control inputs
    subplot(2,2,4)
    plot(z.time, z.control(1,:), 'r-', ...
         z.time, z.control(2,:), 'b-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('T [N]', 'FontSize', 12)
    legend('T_x', 'T_y', 'FontSize', 12)
    title('Entradas de Controle', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    if saveFigures
        mkdir('figures');
        saveas(gcf, ['figures/', figureName, '.fig']);
        saveas(gcf, ['figures/', figureName, '.svg']);
        saveas(gcf, ['figures/', figureName, '.png']);
    end
end 