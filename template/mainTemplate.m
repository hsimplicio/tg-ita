% Main script for template trajectory optimization problem
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
% TODO: Set the number of states and controls for your problem
nx = 2;  % Number of states
nu = 1;  % Number of controls
problem = TrajectoryProblem(nx, nu);

%% Set time bounds
% TODO: Set your desired time bounds
t0 = 0;
tF = 1;
problem.setTimeBounds(t0, tF);

%% Set state bounds
% TODO: Define your state bounds
xLow = [-inf; -inf];  % Lower bounds for each state
xUpp = [inf; inf];    % Upper bounds for each state
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
% TODO: Define your control bounds
uLow = [-inf];  % Lower bounds for each control
uUpp = [inf];   % Upper bounds for each control
problem.setControlBounds(uLow, uUpp);

%% Set parameters
% TODO: Create and set your problem parameters
params = templateParams();
problem.setParameters(params);

%% Set boundary conditions
% TODO: Define your boundary conditions
x0 = [0; 0];  % Initial state
xF = [1; 0];  % Final state
problem.setBoundaryConditions(x0, xF);

%% Set functions
% Set dynamics
problem.setDynamics(@templateDynamics);

% Set objective
problem.setObjective(@boundaryObjective, @pathObjective);

% Set constraints
problem.setConstraints(@boundaryConstraints, @pathConstraints);

%% Set solver options
% TODO: Adjust grid size and solver options as needed
nGrid = [50, 100, 200];  % Number of grid points for each iteration
options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunEvals = 1e5;
problem.setSolverOptions(options, nGrid);

% Optional: Set constraint checking function
problem.setConstraintsCheck(@checkConstraints);

% Optional: Set variable names
% TODO: Set meaningful names for your states and controls
problem.setVariableNames({'x1', 'x2'}, {'u1'});

% Validate problem definition
problem.validate();

% Solve the problem
solution = problem.solveWithTrapezoidalCollocation();

% Plot results
plotResults('Solution', solution(end).z); 