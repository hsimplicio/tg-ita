function stateDerivatives = evtolDynamics(time, state, control, params)
    % Define the dynamics for the EVTOL problem
    %
    % Inputs:
    %   time: [1,n] time vector
    %   state: [5,n] = [posX; posY; velX; velY; energy] = state matrix
    %   control: [2,n] = [thrustX; thrustY] = control matrix
    %   params: parameter struct
    %
    % Outputs:
    %   stateDerivatives: [5,n] = [velX; velY; accelX; accelY; powerRate] = derivative matrix
    %
    arguments
        time double
        state double
        control double
        params struct
    end

    % Get the parameters
    GRAVITY = params.environment.GRAVITY;
    AIR_DENSITY = params.environment.AIR_DENSITY;
    mass = params.aircraft.mass;
    rotorArea = params.aircraft.rotorArea;
    chi = params.aircraft.chi;

    % Assign state variables
    posX = state(1,:);
    posY = state(2,:);
    velX = state(3,:);
    velY = state(4,:);
    energy = state(5,:);
    
    % Assign control variables
    thrustX = control(1,:);
    thrustY = control(2,:);

    % Compute aerodynamic parameters
    V = sqrt(velX.^2 + velY.^2);
    gamma = computeFlightAngle(velX, velY);
    angleToVertical = pi/2 - gamma;  % Angle between velocity and vertical
    angleToHorizontal = gamma;       % Angle between velocity and horizontal

    % Compute the aerodynamic forces
    [lift, drag] = computeLiftDrag(velX, velY, params);

    % Compute the thrust forces
    rotorThrustX = thrustX / 2;
    rotorThrustY = thrustY / 4;

    % Induced velocity calculations
    hoverVelocityX = sqrt(rotorThrustX / (2 * AIR_DENSITY * rotorArea));
    hoverVelocityY = sqrt(rotorThrustY / (2 * AIR_DENSITY * rotorArea));

    inducedVelocityX = computeInducedVelocity(hoverVelocityX, V, angleToHorizontal);
    inducedVelocityY = computeInducedVelocity(hoverVelocityY, V, angleToVertical);

    % Power calculations
    rotorPowerX = rotorThrustX .* inducedVelocityX;
    rotorPowerY = rotorThrustY .* inducedVelocityY;
    armPowerX = 2.0 * rotorPowerX * (1 + chi);
    armPowerY = 4.0 * rotorPowerY * (1 + chi);
    inducedPower = armPowerX + armPowerY;

    forwardPower = thrustX .* V .* sin(angleToHorizontal) + ...
                  thrustY .* V .* sin(angleToVertical);

    % Compute the derivatives
    stateDerivatives = [
        velX;  % posX derivative
        velY;  % posY derivative
        (thrustX - drag .* cos(gamma) - lift .* sin(gamma)) / mass;  % velX derivative
        (thrustY - drag .* sin(gamma) + lift .* cos(gamma) - mass * GRAVITY) / mass;  % velY derivative
        inducedPower + forwardPower;  % energy derivative
    ];
end