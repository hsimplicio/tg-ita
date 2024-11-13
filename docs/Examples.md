# Examples
## Simple Mass Problem
Basic example demonstrating minimum control effort for a mass system.
### Problem Description
- States: position (x), velocity (v)
- Control: force (F)
- Objective: minimize control effort
- Dynamics: double integrator
### Implementation
```matlab
% Create problem instance
nx = 2; % States: position, velocity
nu = 1; % Control: force
problem = TrajectoryProblem(nx, nu);
% Set boundary conditions
x0 = [0; 0]; % Start at origin at rest
xF = [1; 0]; % End at x=1 at rest
problem.setBoundaryConditions(x0, xF);
% Set time conditions
problem.setTimeBoundaryConditions(0, 1);
% Set dynamics
problem.setDynamics(@simpleMassDynamics);
% Set objective (minimize control effort)
problem.setObjective([], @(t,x,u) sum(u.^2,1));
% Set solver options
options = optimoptions('fmincon', 'Display', 'iter');
nGrid = [50, 100]; % Two iterations
problem.setSolverOptions(options, nGrid);
% Solve
solution = problem.solveWithTrapezoidalCollocation();
```
## Brachistochrone Problem
Classical problem of finding the path of fastest descent between two points.
### Problem Description
- States: horizontal position (x), vertical position (y), velocity (v)
- Control: path angle (θ)
- Objective: minimize time
- Dynamics: motion under gravity with path constraint
### Implementation
```matlab
% Create problem instance
nx = 3; % States: [x; y; v]
nu = 1; % Control: [theta]
problem = TrajectoryProblem(nx, nu);
% Set time bounds
problem.setTimeBounds(0, 0, 0, 2);
problem.setTimeBoundaryConditions(0, 1); % Initial guess for final time
% Set state bounds
xLow = [0; -5; 0]; % Lower bounds
xUpp = [5; 0; 15]; % Upper bounds
problem.setStateBounds(xLow, xUpp);
% Set control bounds
uLow = [-pi/2]; % Minimum angle
uUpp = [pi/2]; % Maximum angle
problem.setControlBounds(uLow, uUpp);
% Set boundary conditions
x0 = [0; 0; 0]; % Start at origin with zero velocity
xF = [2; -2; NaN]; % End point (x,y), free velocity
problem.setBoundaryConditions(x0, xF);
% Set functions
problem.setDynamics(@brachistochroneDynamics);
problem.setObjective(@(x0,xF,t0,tF) tF-t0, []); % Minimize time
```
## EVTOL Problem
Complex example of electric vertical takeoff and landing vehicle trajectory optimization.
### Problem Description
- States: position (x,y), velocity (vx,vy), energy (E)
- Controls: thrust components (Tx,Ty)
- Objective: minimize energy consumption
- Constraints: thrust limits, velocity limits, aerodynamics
### Implementation
```matlab
% Create problem instance
problem = TrajectoryProblem(5, 2); % 5 states, 2 controls
% Set time bounds
problem.setTimeBounds(0, 0, 0, 45);
problem.setTimeBoundaryConditions(0, 45);
% Set state bounds
xLow = [0; -10; 0; -5; 0];
xUpp = [1000; 110; 35; 6; 1e8];
problem.setStateBounds(xLow, xUpp);
% Set control bounds
uLow = [0; 0];
uUpp = [1800; 2600];
problem.setControlBounds(uLow, uUpp);
% Set boundary conditions
x0 = [0; 0; 0; 0; 0];
xF = [1000; 100; 25; 0; 1e8]; % Free final energy
problem.setBoundaryConditions(x0, xF);
% Set functions
problem.setDynamics(@evtolDynamics);
problem.setObjective([], @evtolPathObjective);
problem.setConstraints(@evtolBoundaryConstraints, @evtolPathConstraints);
```
## Plotting Results
Each example includes a plotting function to visualize the results:
```matlab
% Plot results from any example
plotResults('Solution Title', solution(end).z, true); % true to save figures
```