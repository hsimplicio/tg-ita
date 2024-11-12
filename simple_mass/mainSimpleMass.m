% Simple mass moving in 1D
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
nx = 2;  % States: [x; v] (position, velocity)
nu = 1;  % Control: [F] (force)
problem = TrajectoryProblem(nx, nu);

%% Set time bounds
t0Low = 0;
t0Upp = 0;
tFLow = 0;
tFUpp = 1;
problem.setTimeBounds(t0Low, t0Upp, tFLow, tFUpp);

%% Set time boundary conditions
t0 = 0;
tF = 1;
problem.setTimeBoundaryConditions(t0, tF);

%% Set state bounds
xLow = [-10; -5];    % Position and velocity lower bounds
xUpp = [10; 5];      % Position and velocity upper bounds
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
uLow = [-50];         % Maximum force in negative direction
uUpp = [50];          % Maximum force in positive direction
problem.setControlBounds(uLow, uUpp);

%% Set parameters
params.MASS = 1.0;   % Mass of the object
problem.setParameters(params);

%% Set boundary conditions
x0 = [0; 0];         % Start at origin with zero velocity
xF = [1; 0];         % End at x=1 with zero velocity
problem.setBoundaryConditions(x0, xF);

%% Set scaling
% problem.setScaling('state', [1; 1]);   % States are already O(1)
% problem.setScaling('control', 1);      % Control is already O(1)
% problem.setScaling('time', 2);         % Time is already O(1)

%% Set functions
problem.setDynamics(@simpleMassDynamics);
problem.setObjective(@boundaryObjective, @pathObjective);
problem.setConstraints(@boundaryConstraints, @pathConstraints);

%% Set solver options
nGrid = [30];  % Start with very few grid points
options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunEvals = 1e5;
% options.Algorithm = 'sqp';
% options.EnableFeasibilityMode = true;
% options.SubproblemAlgorithm = 'cg';
% options.OptimalityTolerance = 1e-8;
% options.ConstraintTolerance = 1e-8;
% options.StepTolerance = 1e-8;

problem.setSolverOptions(options, nGrid);

%% Generate initial guess
zGuess = generateSimpleMassGuess(problem);

%% Solve the problem
solution = problem.solveWithTrapezoidalCollocation(zGuess);

%% Save solution
mkdir('results');
save('results/solution.mat', 'solution');

%% Plot results
plotResults('Simple Mass Solution', solution(end).z, true); 