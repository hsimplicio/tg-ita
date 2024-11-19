% Car lane-change trajectory optimization
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
nx = 4;  % States: [x; y; vx; vy] (position x,y and velocities)
nu = 2;  % Control: [ax; ay] (acceleration in x,y)
problem = TrajectoryProblem(nx, nu);

%% Set time bounds
problem.setTimeBounds(0, 0, 0, 5);  % Fixed time interval [0, 5] seconds
problem.setTimeBoundaryConditions(0, 5);

%% Parameters
params.MAX_ACCEL = 2.0;    % Maximum acceleration [m/s^2]
params.TARGET_VX = 15.0;   % Target forward velocity [m/s]
params.LANE_WIDTH = 3.5;   % Standard lane width [m]
problem.setParameters(params);

%% Set state bounds
xLow = [0; -1; 10; -5];    % x position, y position, x velocity, y velocity
xUpp = [100; 5; 20; 5];
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
uLow = [-params.MAX_ACCEL; -params.MAX_ACCEL];
uUpp = [params.MAX_ACCEL; params.MAX_ACCEL];
problem.setControlBounds(uLow, uUpp);

%% Set boundary conditions
x0 = [0; 0; params.TARGET_VX; 0];          % Start in left lane
xF = [75; params.LANE_WIDTH; params.TARGET_VX; 0];  % End in right lane
problem.setBoundaryConditions(x0, xF);

%% Set functions
problem.setDynamics(@carDynamics);
problem.setObjective([], @pathObjective);  % Minimize acceleration and deviation from target speed
problem.setConstraints(@boundaryConstraints, []);

%% Set solver options
nGrid = [20, 40, 80];  % Start with coarse grid, then refine
options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunEvals = 1e5;
problem.setSolverOptions(options, nGrid);

%% Solve the problem
solution = problem.solveWithTrapezoidalCollocation();

%% Save solution
mkdir('results');
save('results/laneChangeSolution.mat', 'solution');

%% Plot results
plotResults('Lane Change Maneuver', solution(end).z, true);