function plotEvtolResults(figureName, z)
    % Plot the results of the EVTOL trajectory
    figure('Name', figureName);
    
    % Trajectory plot
    subplot(2,2,1)
    plot(z.state(1,:), z.state(2,:), 'b-')
    xlabel('X Position [m]')
    ylabel('Y Position [m]')
    grid on
    title('Flight Path')
    
    % Velocity plot
    subplot(2,2,2)
    plot(z.time, z.state(3,:), 'r-', ...
         z.time, z.state(4,:), 'b-')
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    legend('V_x', 'V_y')
    grid on
    title('Velocities')
    
    % Energy plot
    subplot(2,2,3)
    plot(z.time, z.state(5,:), 'k-')
    xlabel('Time [s]')
    ylabel('Energy [J]')
    grid on
    title('Energy Consumption')
    
    % Control inputs
    subplot(2,2,4)
    plot(z.time, z.control(1,:), 'r-', ...
         z.time, z.control(2,:), 'b-')
    xlabel('Time [s]')
    ylabel('Thrust [N]')
    legend('T_x', 'T_y')
    grid on
    title('Control Inputs')
end 