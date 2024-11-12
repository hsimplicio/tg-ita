function plotResults(figureName, z, saveFigures)
    % Plot the results of your trajectory optimization
    % Inputs:
    %   figureName: string for figure title
    %   z: struct containing solution data
    %   saveFigures: boolean to save the figures
    
    if nargin < 3
        saveFigures = false;
    end

    figure('Name', figureName);
    
    % TODO: Implement your plotting code
    % Example plots:
    subplot(2,2,1)
    plot(z.time, z.state(1,:))
    xlabel('t [s]', 'FontSize', 12)
    ylabel('State 1', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    subplot(2,2,2)
    plot(z.time, z.state(2,:))
    xlabel('t [s]', 'FontSize', 12)
    ylabel('State 2', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);
    
    subplot(2,2,3)
    plot(z.time, z.control(1,:))
    xlabel('t [s]', 'FontSize', 12)
    ylabel('Control', 'FontSize', 12)
    grid on
    set(gca, 'FontSize', 12);

    if saveFigures
        mkdir('results');
        savefig(['results/', figureName, '.fig']);
        saveas(gcf, ['results/', figureName, '.svg']);
        saveas(gcf, ['results/', figureName, '.png']);
    end
end 