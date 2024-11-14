% Brachistochrone problem: Find the path of fastest descent between two points
clear; clc; close all;

% Add path to the TrajectoryProblem class
addpath('..');

%% Create problem instance
nx = 3;  % States: [x; y; v] (horizontal position, vertical position, velocity)
nu = 1;  % Control: [theta] (angle of the path)
problem = TrajectoryProblem(nx, nu);

%% Set time bounds
t0Low = 0;
t0Upp = 0;
tFLow = 0;
tFUpp = 2+1e-6;
problem.setTimeBounds(t0Low, t0Upp, tFLow, tFUpp);

%% Set time boundary conditions
t0 = 0;
tF = 1;  % Initial guess for final time (will be optimized)
problem.setTimeBoundaryConditions(t0, tF);

%% Set state bounds
xLow = [0; -5; 0];     % Lower bounds: x≥0, y≥-5, v≥0
xUpp = [5; 0; 15];      % Upper bounds: x≤5, y≤0, v≤15
problem.setStateBounds(xLow, xUpp);

%% Set control bounds
uLow = [-pi/2];        % Minimum angle
uUpp = [pi/2];         % Maximum angle
problem.setControlBounds(uLow, uUpp);

%% Set parameters
params = brachistochroneParams();
problem.setParameters(params);

%% Set boundary conditions
x0 = [0; 0; 0];        % Start at origin with zero velocity
xF = [5; -5; xUpp(3)]; % End at (5,-5) with free final velocity up to vMax
problem.setBoundaryConditions(x0, xF);

%% Set scaling
% xScale = abs(xF);
% uScale = 1;         % Angle scale
% tScale = sqrt(2*abs(xF(2))/params.GRAVITY);   % Natural time scale

% problem.setScaling('state', xScale);    % Scale states to be O(1)
% problem.setScaling('control', uScale);  % Scale control to be O(1)
% problem.setScaling('time', tScale);     % Scale time to be O(1)

%% Set functions
problem.setDynamics(@brachistochroneDynamics);
problem.setObjective(@boundaryObjective, @pathObjective);
problem.setConstraints(@boundaryConstraints, @pathConstraints);

%% Set solver options
nGrid = [10, 20, 40];  % Number of grid points for each iteration
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

%% Optional: Set constraint checking function
problem.setConstraintsCheck(@checkConstraints);

%% Set variable names
problem.setVariableNames({'x', 'y', 'v'}, {'theta'});

%% Solve the problem
zGuess = generateBrachistochroneGuess(problem);
solution = problem.solveWithTrapezoidalCollocation(zGuess);

%% Display solution
disp('=== Solution Analysis ===');
disp('Time span:');
disp(solution(end).z.timeSpan);

disp('Initial state:');
disp(solution(end).z.state(:,1)');
disp('Target initial state:');
disp(x0');

disp('Final state:');
disp(solution(end).z.state(:,end)');
disp('Target final state:');
disp(xF');

% Check energy conservation
E = 0.5 * solution(end).z.state(3,:).^2 + params.GRAVITY * solution(end).z.state(2,:);
disp('Energy variation:');
disp(['Max: ', num2str(max(E) - min(E))]);

% Save the solution
mkdir('results');
save('results/solution.mat', 'solution');

%% Plot results
plotResults('Brachistochrone Solution', solution(end).z, true);

%% Compare with analytical solution
compareWithAnalytical(solution(end).z, true);
