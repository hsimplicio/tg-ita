function plotResults(figureName, z, saveFigures)
    % Plots the results of the simple mass dynamics
    % figureName: name of the figure
    % z: solution structure
    % saveFigures: boolean to save the figures

    if nargin < 3
        saveFigures = false;
    end

    figure('Name', figureName);

    % Position over time
    subplot(3,1,1)
    plot(z.time, z.state(1,:), 'b-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('x [m]', 'FontSize', 12)
    title('Posição', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    % Velocity over time
    subplot(3,1,2)
    plot(z.time, z.state(2,:), 'r-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('v [m/s]', 'FontSize', 12)
    grid on
    title('Velocidade', 'FontSize', 12)
    set(gca, 'FontSize', 12);
    
    % Control force over time
    subplot(3,1,3)
    plot(z.time, z.control(1,:), 'g-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('F [N]', 'FontSize', 12)
    grid on
    title('Força', 'FontSize', 12)
    set(gca, 'FontSize', 12);

    if saveFigures
        mkdir('figures');
        saveas(gcf, ['figures/', figureName, '.fig']);
        saveas(gcf, ['figures/', figureName, '.svg']);
        saveas(gcf, ['figures/', figureName, '.png']);
    end

end 