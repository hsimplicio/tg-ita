function dx = templateDynamics(time, state, control, params)
    % Define the dynamics for your problem
    % Inputs:
    %   time: [1,n] time vector
    %   state: [nx,n] state matrix
    %   control: [nu,n] control matrix
    %   params: parameter struct
    % Outputs:
    %   dx: [nx,n] state derivative matrix
    
    arguments
        time double
        state double
        control double
        params struct
    end
    
    % TODO: Implement your system dynamics
    % Example for a double integrator:
    dx = [
        state(2,:);                % dx1/dt = x2
        control(1,:)               % dx2/dt = u1
    ];
end 