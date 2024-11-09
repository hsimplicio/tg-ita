%% Main script for EVTOL trajectory optimization
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
problem = TrajectoryProblem(5, 2);  % 5 states, 2 controls

%% Set time bounds
t0 = 0;
tF = 45;
problem.setTimeBounds(t0, tF);

%% Set state bounds
xLow = [0; -10; 0; -5; 0];
xUpp = [1000; 110; 35; 6; 1e8];
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
uLow = [0; 0];
uUpp = [1800; 2600];
problem.setControlBounds(uLow, uUpp);

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
nGrid = [50, 80, 150, 190, 250];
options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunEvals = 1e5;
options.EnableFeasibilityMode = true;
options.SubproblemAlgorithm = 'cg';
problem.setSolverOptions(options, nGrid);

% Optional: Set constraint checking function
problem.setConstraintsCheck(@checkConstraints);

% Optional: Set variable names
problem.setVariableNames({'sx', 'sy', 'vx', 'vy', 'E'}, {'Tx', 'Ty'});

% Validate problem definition
problem.validate();

% Optional: Get initial guess
zGuess = physicalInitialGuess(problem, true);

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

% Save the final solution
z = solution(end).z;
% save(['data/solution-base-t', num2str(tF), '.mat'], 'z');

% Plot results
plotEvtolResults('Final Solution', z);
