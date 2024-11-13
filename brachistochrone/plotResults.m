function plotResults(figureName, z, saveFigures)
    % Plots the results of the brachistochrone problem
    % figureName: name of the figure
    % z: solution structure
    % saveFigures: boolean to save the figures

    if nargin < 3
        saveFigures = false;
    end

    figure('Name', figureName);
    
    % Path in x-y plane
    subplot(2,2,1)
    plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2)
    xlabel('x [m]', 'FontSize', 12)
    ylabel('y [m]', 'FontSize', 12)
    title('Trajetória', 'FontSize', 12)
    grid on
    axis equal
    set(gca, 'FontSize', 12);
    
    % Velocity over time
    subplot(2,2,2)
    plot(z.time, z.state(3,:), 'r-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('v [m/s]', 'FontSize', 12)
    title('Perfil de Velocidade', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    % Control angle over time
    subplot(2,2,3)
    plot(z.time, z.control(1,:)*180/pi, 'g-', 'LineWidth', 2)
    xlabel('t [s]', 'FontSize', 12)
    ylabel('\theta [deg]', 'FontSize', 12)
    title('Entrada de Controle', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    % Phase portrait
    subplot(2,2,4)
    plot(z.state(1,:), z.state(3,:), 'b-', 'LineWidth', 2)
    xlabel('x [m]', 'FontSize', 12)
    ylabel('v [m/s]', 'FontSize', 12)
    title('Diagrama de Fase', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    if saveFigures
        mkdir('figures');
        saveas(gcf, ['figures/', figureName, '.fig']);
        saveas(gcf, ['figures/', figureName, '.svg']);
        saveas(gcf, ['figures/', figureName, '.png']);
    end
end 