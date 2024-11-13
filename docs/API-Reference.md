# API Reference
## TrajectoryProblem Class
The main class handling all aspects of trajectory optimization problems.
### Constructor
```matlab
problem = TrajectoryProblem(nx, nu)
```
- `nx`: Number of state variables
- `nu`: Number of control variables
### Required Methods
#### Setting Problem Structure
```matlab
setBoundaryConditions(x0, xF) % Set initial and final states
setTimeBoundaryConditions(t0, tF) % Set time bounds
setDynamics(hDynamics) % Set system dynamics
setObjective(hBoundary, hPath) % Set objective functions
setSolverOptions(options, nGrid) % Set solver options
```
### Optional Methods
```matlab
setTimeBounds(t0Low, t0Upp, tFLow, tFUpp) % Set time bounds
setStateBounds(xLow, xUpp) % Set state bounds
setControlBounds(uLow, uUpp) % Set control bounds
setScaling(type, factors) % Set scaling factors
setConstraints(hBoundary, hPath) % Set constraints
setParameters(parameters) % Set parameters
```
### Solution Method
```matlab
solution = solveWithTrapezoidalCollocation([guess])
```