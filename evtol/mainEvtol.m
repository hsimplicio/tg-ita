%% Main script for EVTOL trajectory optimization
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
problem = TrajectoryProblem(5, 2);  % 5 states, 2 controls

%% Set time bounds
t0Low = 0;
t0Upp = 0;
tFLow = 0;
tFUpp = 45;
problem.setTimeBounds(t0Low, t0Upp, tFLow, tFUpp);

%% Set time boundary conditions
t0 = 0;
tF = 45;
problem.setTimeBoundaryConditions(t0, tF);

%% Set state bounds
xLow = [0; -10; 0; -5; 0];
xUpp = [1000; 110; 35; 6; 1e8];
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
uLow = [0; 0];
uUpp = [1800; 2600];
problem.setControlBounds(uLow, uUpp);

%% Set scaling
% problem.setScaling('time', tF - t0);
% problem.setScaling('state', xUpp - xLow);
% problem.setScaling('control', uUpp - uLow);

%% Set parameters
problem.setParameters(evtolParams());

%% Set boundary conditions
x0 = [0; 0; 0; 0; 0];
xF = [1000; 100; 25; 0; 1e8];  % Free final energy
problem.setBoundaryConditions(x0, xF);

%% Set functions
% Set dynamics
problem.setDynamics(@evtolDynamics);

% Set objective
hBoundaryObjective = @boundaryObjective;    % φ(·) - Mayer Term
hPathObjective = [];                        % L(·) - integrand of Lagrange Term
problem.setObjective(hBoundaryObjective, hPathObjective);

% Set constraints
hBoundaryConstraint = @boundaryConstraints;  % h(·) - Boundary Constraints
hPathConstraint = [];                       % g(·) - Path Constraints
problem.setConstraints(hBoundaryConstraint, hPathConstraint);

%% Set solver options
nGrid = [10, 20, 40, 80];
options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunEvals = 1e5;
options.Algorithm = 'sqp';
options.EnableFeasibilityMode = true;
options.SubproblemAlgorithm = 'cg';
options.FiniteDifferenceType = 'central';  % More accurate gradients
options.FiniteDifferenceStepSize = 1e-6;   % Smaller step size
options.OptimalityTolerance = 1e-6;        % Tighter tolerance
options.ConstraintTolerance = 1e-6;        % Tighter tolerance
options.StepTolerance = 1e-10;             % Smaller steps
options.MaxIterations = 1000;              % Increase if needed
options.ScaleProblem = true;               % Add scaling
options.HessianApproximation = 'bfgs';     % Use BFGS approximation

problem.setSolverOptions(options, nGrid);

% Optional: Set constraint checking function
problem.setConstraintsCheck(@checkConstraints);

% Optional: Set variable names
problem.setVariableNames({'sx', 'sy', 'vx', 'vy', 'E'}, {'Tx', 'Ty'});

% Validate problem definition
problem.validate();

% Optional: Get initial guess
zGuess = physicalInitialGuess(problem);

% Solve the problem
solution = problem.solveWithTrapezoidalCollocation(zGuess);

% Check constraints
violations = solution(end).violations;
% Display constraint violations
if ~isempty(fieldnames(violations.state))
    disp('State Violations:')
    disp(violations.state)
end

if ~isempty(fieldnames(violations.control))
    disp('Control Violations:')
    disp(violations.control)
end

if ~isempty(fieldnames(violations.physical))
    disp('Physical Consistency:')
    disp(violations.physical.velocity)
    disp(violations.physical.flightPathAngle)
end

disp(solution(end).output)

% Save the solution
mkdir('results');
save(['results/solution-base-t', num2str(tF), '.mat'], 'solution');

% Plot results
plotResults('Final Solution', solution(end).z, true);
