% Pendulum swing-up trajectory optimization
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
nx = 2;  % States: [theta; omega] (angle, angular velocity)
nu = 1;  % Control: [tau] (torque)
problem = TrajectoryProblem(nx, nu);

%% Set time bounds
problem.setTimeBounds(0, 0, 0, 3);  % Fixed time interval [0, 3]
problem.setTimeBoundaryConditions(0, 3);

%% Set state bounds
% Angle bounds: ±4π (allow multiple rotations)
% Angular velocity bounds: ±8π rad/s
xLow = [-4*pi; -8*pi];
xUpp = [4*pi; 8*pi];
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
uLow = [-2];  % Maximum negative torque
uUpp = [2];   % Maximum positive torque
problem.setControlBounds(uLow, uUpp);

%% Set parameters
params.MASS = 1.0;      % Mass [kg]
params.LENGTH = 0.5;    % Length [m]
params.GRAVITY = 9.81;  % Gravity [m/s^2]
params.DAMPING = 0.1;   % Damping coefficient
problem.setParameters(params);

%% Set boundary conditions
x0 = [pi; 0];    % Start hanging down (θ = π) at rest
xF = [0; 0];     % End pointing up (θ = 0) at rest
problem.setBoundaryConditions(x0, xF);

%% Set functions
problem.setDynamics(@pendulumDynamics);
problem.setObjective([], @pathObjective);  % Minimize control effort
problem.setConstraints([], []);  % No additional constraints needed

%% Set solver options
nGrid = [30, 60];  % Start with coarse grid, then refine
options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunctionEvaluations = 1e5;
problem.setSolverOptions(options, nGrid);

%% Set variable names
problem.setVariableNames({'theta', 'omega'}, {'tau'});

%% Solve the problem
solution = problem.solveWithTrapezoidalCollocation();

%% Save solution
mkdir('results');
save('results/solution.mat', 'solution');

%% Plot results
plotResults('Pendulum Swing-up', solution(end).z, true); 