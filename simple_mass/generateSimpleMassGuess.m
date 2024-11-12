function zGuess = generateSimpleMassGuess(problem)
    % Extract problem parameters
    [x0, xF] = problem.getBoundaryConditions();
    nGrid = problem.getGridSize();
    
    % Linear interpolation for states
    state = [
        linspace(x0(1), xF(1), nGrid(1));  % Position
        linspace(x0(2), xF(2), nGrid(1))   % Velocity
    ];
    
    % Linear interpolation for control
    control = linspace(1, -1, nGrid(1));
    
    % Combine states and control
    zGuess = [state; control];
end 