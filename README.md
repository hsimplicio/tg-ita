# Trajectory Optimization Library

A MATLAB library for solving trajectory optimization problems using trapezoidal collocation. The library transforms optimal control problems into nonlinear programming problems that can be solved using MATLAB's `fmincon` optimizer.

## Features

- Trapezoidal collocation method
- Support for multiple solver iterations with grid refinement
- Automatic scaling capabilities
- Flexible constraint handling
- Built-in examples and templates

## Installation

1. Clone this repository
2. Add the library directory to your MATLAB path
3. Run the examples to verify installation

## Basic Usage

```matlab
% Create problem instance
nx = 2;  % Number of states
nu = 1;  % Number of controls
problem = TrajectoryProblem(nx, nu);

% Set basic problem properties
problem.setTimeBoundaryConditions(0, 1);           % Time span [0,1]
problem.setBoundaryConditions(x0, xF);             % Initial and final states
problem.setStateBounds(xLow, xUpp);                % State bounds
problem.setControlBounds(uLow, uUpp);              % Control bounds

% Set required functions
problem.setDynamics(@systemDynamics);              % System dynamics
problem.setObjective(@boundaryObj, @pathObj);      % Objective functions
problem.setConstraints(@boundaryConst, @pathConst);% Optional constraints

% Set solver options
options = optimoptions('fmincon', 'Display', 'iter');
nGrid = [50, 100, 200];  % Multiple iterations with grid refinement
problem.setSolverOptions(options, nGrid);

% Solve the problem
solution = problem.solveWithTrapezoidalCollocation();
```

## Required Functions

### System Dynamics
```matlab
function dx = systemDynamics(time, state, control, params)
    % Returns state derivatives
    dx = [state(2,:); control(1,:)];  % Example: double integrator
end
```

### Objective Functions
```matlab
function cost = boundaryObjective(x0, xF, t0, tF, params)
    % Mayer term (terminal cost)
    cost = tF - t0;  % Example: minimum time
end

function L = pathObjective(time, state, control, params)
    % Lagrange term (running cost)
    L = sum(control.^2, 1);  % Example: minimum control effort
end
```

## Examples

The library includes several example problems:

1. **Simple Mass**: Basic 1D mass movement problem
2. **Brachistochrone**: Classical problem of finding fastest descent path
3. **EVTOL**: Complex trajectory optimization for an electric vertical takeoff and landing vehicle

Run any example by navigating to its directory and running the main script:

```matlab
cd examples/brachistochrone
mainBrachistochrone
```

## Advanced Features

- Multiple solver iterations with automatic grid refinement
- Constraint violation checking
- State and control scaling
- Custom parameter handling
- Detailed solution analysis

## Documentation

See the [detailed documentation](docs/Home) for:
- Complete API reference
- Function specifications
- Example explanations
- Best practices
- Troubleshooting guide

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

<!-- ## License

[Add your chosen license here]

## Citation

If you use this library in your research, please cite:
[Add citation information here]

## Contact

[Add contact information here]

## Acknowledgments

[Add acknowledgments here] -->