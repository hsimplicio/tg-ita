function zGuess = physicalInitialGuess(problem, plotFlag)
    % Generate a physics-based initial guess for the EVTOL problem
    
    % Extract problem parameters
    params = problem.getParameters();
    mass = params.aircraft.mass;
    g = params.environment.GRAVITY;
    
    % Extract time bounds, boundary conditions, and grid size
    [t0, tF] = problem.getTimeBounds();
    [x0, xF] = problem.getBoundaryConditions();
    nGrid = problem.getGridSize();

    % Generate time vector and relative time
    time = linspace(t0, tF, nGrid(1));
    tRel = time - t0;  % Shift time to start at 0
    duration = tF - t0;
    
    % Calculate trajectory using relative time
    posX = x0(1) + x0(3)*tRel + (3*(xF(1)-x0(1))/duration^2 - 2*x0(3)/duration - xF(3)/duration)*tRel.^2 + ...
         (-2*(xF(1)-x0(1))/duration^3 + (x0(3)+xF(3))/duration^2)*tRel.^3;
    posY = x0(2) + x0(4)*tRel + (3*(xF(2)-x0(2))/duration^2 - 2*x0(4)/duration - xF(4)/duration)*tRel.^2 + ...
         (-2*(xF(2)-x0(2))/duration^3 + (x0(4)+xF(4))/duration^2)*tRel.^3;
    velX = gradient(posX, time);  % Note: gradient still uses original time step
    velY = gradient(posY, time);
    
    % Add validation
    assert(all(isfinite(velX)), 'Non-finite values in velX');
    assert(all(isfinite(velY)), 'Non-finite values in velY');
    
    % Calculate minimum thrust needed to maintain trajectory
    [lift, drag] = computeLiftDrag(velX, velY, params);
    gamma = computeFlightAngle(velX, velY);
    
    % Solve for required thrust
    thrustX = drag .* cos(gamma) + lift .* sin(gamma);
    thrustY = drag .* sin(gamma) - lift .* cos(gamma) + mass .* g;
    
    % Calculate power at this point
    state = [posX; posY; velX; velY; zeros(1,length(time))];
    control = [thrustX; thrustY];
    dx = evtolDynamics(time, state, control, params);
    dE = dx(5,:);

    % Integrate power to get energy
    energy = x0(5) + cumtrapz(time, dE);

    state(5,:) = energy;

    % Combine into guess
    zGuess = [state; control];

    % Plot if requested
    if plotFlag
        guess = struct();
        guess.time = time;
        guess.state = state;
        guess.control = control;
        plotEvtolResults('Initial Guess', guess);
    end
end
