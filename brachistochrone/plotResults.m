function plotResults(figureName, z)
    figure('Name', figureName);
    
    % Path in x-y plane
    subplot(2,2,1)
    plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2)
    xlabel('X Position [m]')
    ylabel('Y Position [m]')
    grid on
    title('Brachistochrone Path')
    axis equal
    
    % Velocity over time
    subplot(2,2,2)
    plot(z.time, z.state(3,:), 'r-', 'LineWidth', 2)
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    grid on
    title('Velocity Profile')
    
    % Control angle over time
    subplot(2,2,3)
    plot(z.time, z.control(1,:)*180/pi, 'g-', 'LineWidth', 2)
    xlabel('Time [s]')
    ylabel('Path Angle [deg]')
    grid on
    title('Control Input')
    
    % Phase portrait
    subplot(2,2,4)
    plot(z.state(1,:), z.state(3,:), 'b-', 'LineWidth', 2)
    xlabel('Position [m]')
    ylabel('Velocity [m/s]')
    grid on
    title('Phase Portrait')
end 