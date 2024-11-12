function zGuess = generateBrachistochroneGuess(problem, plotFlag)
    % Extract problem parameters
    params = problem.getParameters();
    g = params.GRAVITY;
    
    % Extract time bounds, boundary conditions, and grid size
    [t0, tF] = problem.getTimeBounds();
    [x0, xF] = problem.getBoundaryConditions();
    nGrid = problem.getGridSize();

    time = linspace(t0, tF, nGrid(1));
    
    % Initial and final positions
    sx0 = x0(1); sy0 = x0(2);
    sxf = xF(1); syf = xF(2);
    
    % Generate parabolic path as initial guess
    sx = linspace(sx0, sxf, nGrid(1));
    %
    % x - sx0 = a * (y - sy0)^2
    %
    a = (sxf - sx0) / (syf - sy0)^2;
    sy = sy0 + sign(syf - sy0) * sqrt((sx - sx0) / a);
    
    % Estimate velocity (using energy conservation)
    v = sqrt(2 * g * (sy0 - sy));
    
    % Estimate control angle
    dsx = gradient(sx, time);
    dsy = gradient(sy, time);
    theta = atan2(dsy, dsx);
    
    state = [sx; sy; v];
    control = theta;
    zGuess = [state; control];

    % Plot if requested
    if plotFlag
        guess = struct();
        guess.time = time;
        guess.state = state;
        guess.control = control;
        plotResults('Initial Guess', guess);
        compareWithAnalytical(guess);
    end
end
