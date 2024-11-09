function plotResults(figureName, z)
    % Plot the results of your trajectory optimization
    % Inputs:
    %   figureName: string for figure title
    %   z: struct containing solution data
    
    figure('Name', figureName);
    
    % TODO: Implement your plotting code
    % Example plots:
    subplot(2,2,1)
    plot(z.time, z.state(1,:))
    xlabel('Time')
    ylabel('State 1')
    grid on
    
    subplot(2,2,2)
    plot(z.time, z.state(2,:))
    xlabel('Time')
    ylabel('State 2')
    grid on
    
    subplot(2,2,3)
    plot(z.time, z.control(1,:))
    xlabel('Time')
    ylabel('Control')
    grid on
end 