# Function Specifications
## Required Functions
### System Dynamics
```matlab
function dx = dynamics(time, state, control, params)
arguments
time double % [1,n] Time points
state double % [nx,n] State trajectory
control double % [nu,n] Control trajectory
params struct % Parameter structure
end
% Return state derivatives [nx,n]
dx = [...];
end
```
### Objective Functions
#### Boundary (Mayer) Term
```matlab
function cost = boundaryObjective(x0, xF, t0, tF, params)
arguments
x0 double % [nx,1] Initial state
xF double % [nx,1] Final state
t0 double % Initial time
tF double % Final time
params struct % Parameter structure
end
% Return scalar cost
cost = ...;
end
```
#### Path (Lagrange) Term
```matlab
function L = pathObjective(time, state, control, params)
arguments
time double % [1,n] Time points
state double % [nx,n] State trajectory
control double % [nu,n] Control trajectory
params struct % Parameter structure
end
% Return instantaneous cost [1,n]
L = ...;
end
```
## Optional Functions
### Constraint Functions
#### Boundary Constraints
```matlab
function [c, ceq] = boundaryConstraints(x0, xF, t0, tF, params)
arguments
x0 double % [nx,1] Initial state
xF double % [nx,1] Final state
t0 double % Initial time
tF double % Final time
params struct % Parameter structure
end
% c: Inequality constraints (c <= 0)
% ceq: Equality constraints (ceq = 0)
c = [...]; % [m,1] vector
ceq = [...]; % [p,1] vector
end
```
#### Path Constraints
```matlab
function [c, ceq] = pathConstraints(time, state, control, params)
arguments
time double % [1,n] Time points
state double % [nx,n] State trajectory
control double % [nu,n] Control trajectory
params struct % Parameter structure
end
% c: Inequality constraints [m,n]
% ceq: Equality constraints [p,n]
c = [...]; % Each column represents constraints at a time point
ceq = [...]; % Each column represents constraints at a time point
end
```
## Return Value Specifications
### Solution Structure
The `solveWithTrapezoidalCollocation` method returns a structure array with results from all iterations:
```matlab
solution(nIter) = struct();
solution(i).nGrid % Number of grid points in iteration i
solution(i).z.timeSpan % Time span [t0, tF]
solution(i).z.time % Time grid points
solution(i).z.state % State trajectory [nx,nGrid]
solution(i).z.control % Control trajectory [nu,nGrid]
solution(i).z.derivatives% State derivatives [nx,nGrid]
solution(i).fval % Objective function value
solution(i).exitflag % Solver exit flag
solution(i).output % Solver output information
```