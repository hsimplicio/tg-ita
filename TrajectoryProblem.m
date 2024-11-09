classdef TrajectoryProblem < handle
    % Class to define a trajectory optimization problem
    
    properties (SetAccess = private)
        % Problem dimensions
        nx  % Number of states
        nu  % Number of controls
        
        % Time properties
        t0 {mustBeNumeric}  % Initial time
        tF {mustBeNumeric}  % Final time
        
        % Boundary conditions
        x0  % Initial state
        xF  % Final state
        boundaryConditions  % Boundary conditions struct
        
        % State and control bounds
        xLow  % Lower state bounds
        xUpp  % Upper state bounds
        uLow  % Lower control bounds
        uUpp  % Upper control bounds
        
        % Function handles
        dynamics  % System dynamics
        constraints  % Constraint function
        objective  % Objective function
        constraintsCheck  % Function handle for problem-specific constraint checking
        
        % Constraint functions
        boundaryConstraints  % Boundary constraints function
        pathConstraints  % Path constraints function
        
        % Objective functions
        boundaryObjective  % Mayer term
        pathObjective  % Lagrange term

        % Parameters
        parameters  % Problem parameters
        
        % Solver properties
        nGrid = []  % Array of grid points for each iteration
        solverOptions = {}  % Cell array of options for each iteration
        
    end
    
    properties
        % Optional properties
        description = ''  % Problem description
        stateNames = {}  % Names of state variables
        controlNames = {}  % Names of control variables
    end
    
    methods
        function obj = TrajectoryProblem(nx, nu)
            % Constructor
            if nargin < 2
                error('Must specify number of states and controls');
            end
            
            obj.nx = nx;
            obj.nu = nu;
            
            % Initialize with empty arrays
            obj.x0 = zeros(nx, 1);
            obj.xF = zeros(nx, 1);
            obj.xLow = -inf(nx, 1);
            obj.xUpp = inf(nx, 1);
            obj.uLow = -inf(nu, 1);
            obj.uUpp = inf(nu, 1);
            
            % Default time interval
            obj.t0 = 0;
            obj.tF = 1;
        end
        
        function setTimeBounds(obj, t0, tF)
            % Set time bounds
            validateattributes(t0, {'numeric'}, {'scalar'});
            validateattributes(tF, {'numeric'}, {'scalar', '>', t0});
            obj.t0 = t0;
            obj.tF = tF;
        end
        
        function setBoundaryConditions(obj, x0, xF)
            % Set boundary conditions
            validateattributes(x0, {'numeric'}, {'vector', 'numel', obj.nx});
            validateattributes(xF, {'numeric'}, {'vector', 'numel', obj.nx});
            obj.x0 = x0(:);  % Ensure column vector
            obj.xF = xF(:);
            obj.boundaryConditions = struct('x0', x0, 'xF', xF);
        end
        
        function setStateBounds(obj, xLow, xUpp)
            % Set state bounds
            validateattributes(xLow, {'numeric'}, {'vector', 'numel', obj.nx});
            validateattributes(xUpp, {'numeric'}, {'vector', 'numel', obj.nx});
            assert(all(xLow <= xUpp), 'Lower bounds must be <= upper bounds');
            obj.xLow = xLow(:);
            obj.xUpp = xUpp(:);
        end
        
        function setControlBounds(obj, uLow, uUpp)
            % Set control bounds
            validateattributes(uLow, {'numeric'}, {'vector', 'numel', obj.nu});
            validateattributes(uUpp, {'numeric'}, {'vector', 'numel', obj.nu});
            assert(all(uLow <= uUpp), 'Lower bounds must be <= upper bounds');
            obj.uLow = uLow(:);
            obj.uUpp = uUpp(:);
        end
        
        function setDynamics(obj, hDynamics)
            % Set dynamics function
            validateattributes(hDynamics, {'function_handle'}, {});
            % Test the function with dummy inputs
            time_test = zeros(1,1);
            x_test = zeros(obj.nx, 1);
            u_test = zeros(obj.nu, 1);
            try
                dx = hDynamics(time_test, x_test, u_test, obj.parameters);
                validateattributes(dx, {'numeric'}, {'vector', 'numel', obj.nx});
            catch ME
                error('Invalid dynamics function: %s', ME.message);
            end
            obj.dynamics = hDynamics;
        end
        
        function setObjective(obj, hBoundaryObjective, hPathObjective)
            % Set objective functions
            % Both inputs are optional - pass [] to skip
            
            % Validate boundary objective if provided
            if ~isempty(hBoundaryObjective)
                validateattributes(hBoundaryObjective, {'function_handle'}, {});
                try
                    val = hBoundaryObjective(zeros(obj.nx,1), zeros(obj.nx,1), obj.t0, obj.tF, obj.parameters);
                    validateattributes(val, {'numeric'}, {'scalar'});
                catch ME
                    error('Invalid boundary objective function: %s', ME.message);
                end
                obj.boundaryObjective = @(x0, xF, t0, tF) hBoundaryObjective(x0, xF, t0, tF, obj.parameters);
            else
                obj.boundaryObjective = [];
            end
            
            % Validate path objective if provided
            if ~isempty(hPathObjective)
                validateattributes(hPathObjective, {'function_handle'}, {});
                try
                    time_test = [obj.t0, obj.tF];
                    val = hPathObjective(time_test, zeros(obj.nx,2), zeros(obj.nu,2), obj.parameters);
                    validateattributes(val, {'numeric'}, {'scalar'});
                catch ME
                    error('Invalid path objective function: %s', ME.message);
                end
                obj.pathObjective = @(time, state, control) hPathObjective(time, state, control, obj.parameters);
            else
                obj.pathObjective = [];
            end
            
            % Create combined objective function
            obj.objective = @(time, state, control) evaluateObjective(...
                time, state, control, ...
                obj.boundaryObjective, ...
                obj.pathObjective);
        end

        function setConstraints(obj, hBoundaryConstraints, hPathConstraints)
            % Set constraint functions
            % Both inputs are optional - pass [] to skip
            
            % Validate boundary constraints if provided
            if ~isempty(hBoundaryConstraints)
                validateattributes(hBoundaryConstraints, {'function_handle'}, {});
                try
                    [c, ceq] = hBoundaryConstraints(zeros(obj.nx,1), zeros(obj.nx,1), obj.t0, obj.tF, obj.boundaryConditions);
                    if ~isempty(c)
                        validateattributes(c, {'numeric'}, {'vector'});
                    end
                    if ~isempty(ceq)
                        validateattributes(ceq, {'numeric'}, {'vector'});
                    end
                catch ME
                    error('Invalid boundary constraints function: %s', ME.message);
                end
                obj.boundaryConstraints = @(x0, xF, t0, tF) hBoundaryConstraints(x0, xF, t0, tF, obj.boundaryConditions);
            else
                obj.boundaryConstraints = [];
            end
            
            % Validate path constraints if provided
            if ~isempty(hPathConstraints)
                validateattributes(hPathConstraints, {'function_handle'}, {});
                try
                    time_test = [obj.t0, obj.tF];
                    [c, ceq] = hPathConstraints(time_test, zeros(obj.nx,2), zeros(obj.nu,2), obj.parameters);
                    if ~isempty(c)
                        validateattributes(c, {'numeric'}, {'vector'});
                    end
                    if ~isempty(ceq)
                        validateattributes(ceq, {'numeric'}, {'vector'});
                    end
                catch ME
                    error('Invalid path constraints function: %s', ME.message);
                end
                obj.pathConstraints = @(time, state, control) hPathConstraints(time, state, control, obj.parameters);
            else
                obj.pathConstraints = [];
            end
            
            % Create combined constraints function
            obj.constraints = @(time, state, control) evaluateConstraints(...
                time, state, control, ...
                obj.dynamics, ...
                @computeDefects, ...
                obj.pathConstraints, ...
                obj.boundaryConstraints, ...
                obj.parameters);
        end
        
        function setParameters(obj, parameters)
            % Set problem parameters
            obj.parameters = parameters;
        end
        
        function setVariableNames(obj, stateNames, controlNames)
            % Set names for variables (optional)
            if nargin > 1 && ~isempty(stateNames)
                validateattributes(stateNames, {'cell'}, {'numel', obj.nx});
                obj.stateNames = stateNames;
            end
            if nargin > 2 && ~isempty(controlNames)
                validateattributes(controlNames, {'cell'}, {'numel', obj.nu});
                obj.controlNames = controlNames;
            end
        end
        
        function setSolverOptions(obj, options, nGrid)
            % Set solver options and grid points
            % options: either a single optimoptions('fmincon') object or cell array of options for each iteration
            % nGrid: array of grid points for each iteration
            
            validateattributes(nGrid, {'numeric'}, {'vector', 'positive', 'integer'});
            
            if ~iscell(options)
                % If single options provided, replicate for all iterations
                validateattributes(options, {'optim.options.Fmincon'}, {});
                obj.solverOptions = repmat({options}, 1, length(nGrid));
            else
                % If cell array provided, validate length matches nGrid
                validateattributes(options, {'cell'}, {'numel', length(nGrid)});
                cellfun(@(opt) validateattributes(opt, {'optim.options.Fmincon'}, {}), options);
                obj.solverOptions = options;
            end
            
            obj.nGrid = nGrid;
        end
        
        function setConstraintsCheck(obj, hConstraintsCheck)
            % Set function handle for problem-specific constraint checking
            validateattributes(hConstraintsCheck, {'function_handle'}, {});
            
            % Test the function with dummy inputs
            time_test = linspace(obj.t0, obj.tF, 2);
            state_test = zeros(obj.nx, 2);
            control_test = zeros(obj.nu, 2);
            try
                violations = hConstraintsCheck(time_test, state_test, control_test, obj.parameters);
                validateattributes(violations, {'struct'}, {});
            catch ME
                error('Invalid constraints check function: %s', ME.message);
            end
            
            obj.constraintsCheck = hConstraintsCheck;
        end
        
        function valid = validate(obj)
            % Validate that all required properties are set
            
            % Check each requirement individually
            hasDynamics = ~isempty(obj.dynamics);
            hasObjective = ~isempty(obj.objective);
            hasFiniteBounds = ~any(isinf([obj.xLow; obj.xUpp; obj.uLow; obj.uUpp]));
            hasGrid = ~isempty(obj.nGrid);
            hasOptions = ~isempty(obj.solverOptions);
            gridMatchesOptions = length(obj.nGrid) == length(obj.solverOptions);
            
            % Combine all checks
            valid = hasDynamics && hasObjective && hasFiniteBounds && ...
                    hasGrid && hasOptions && gridMatchesOptions;
            
            % Provide detailed warning if invalid
            if ~valid
                warning('TrajectoryProblem:Incomplete', ...
                    'Problem definition is incomplete. Missing requirements:\n%s%s%s%s%s%s', ...
                    conditional_msg(~hasDynamics, '- Dynamics function not set'), ...
                    conditional_msg(~hasObjective, '- Objective function not set'), ...
                    conditional_msg(~hasFiniteBounds, '- Infinite bounds detected'), ...
                    conditional_msg(~hasGrid, '- Grid points not set'), ...
                    conditional_msg(~hasOptions, '- Solver options not set'), ...
                    conditional_msg(~gridMatchesOptions, '- Number of grid points does not match number of solver options'));
            end
            
            function msg = conditional_msg(condition, message)
                if condition
                    msg = message;
                else
                    msg = '';
                end
            end
        end
        
        function info = summarize(obj)
            % Generate problem summary
            info = struct();
            info.numStates = obj.nx;
            info.numControls = obj.nu;
            info.timespan = [obj.t0, obj.tF];
            info.stateBounds = [obj.xLow, obj.xUpp];
            info.controlBounds = [obj.uLow, obj.uUpp];
            if ~isempty(obj.stateNames)
                info.stateNames = obj.stateNames;
            end
            if ~isempty(obj.controlNames)
                info.controlNames = obj.controlNames;
            end
        end
        
        function guess = generateInitialGuess(obj)
            % Generate linear interpolation between boundary conditions if no guess provided
            
            % Validate boundary conditions exist
            if isempty(obj.x0) || isempty(obj.xF)
                error('TrajectoryProblem:NoBoundaryConditions', ...
                    'Either pass initial guess or set boundary conditions to generate a linear interpolation initial guess.');
            end
            
            % Linear interpolation for states
            time = linspace(obj.t0, obj.tF, obj.nGrid(1));
            x_guess = interp1([obj.t0, obj.tF]', [obj.x0, obj.xF]', time')';
            
            % Initialize controls to maintain hover
            % This is a simple initialization - might need to be customized for different problems
            u_guess = zeros(obj.nu, obj.nGrid(1));
            
            % Combine state and control guesses
            guess = [x_guess; u_guess];
        end
        
        function solution = solveWithTrapezoidalCollocation(obj, zGuess)
            % Solve the trajectory optimization problem with trapezoidal collocation
            if ~obj.validate()
                error('TrajectoryProblem:InvalidProblem', ...
                    'Problem is not completely defined');
            end
            
            nIter = length(obj.nGrid);
            solution(nIter) = struct();
            
            for i = 1:nIter
                disp(['Iteration ', num2str(i), ' of ', num2str(nIter)]);
                time = linspace(obj.t0, obj.tF, obj.nGrid(i));
                
                % Generate or interpolate initial guess
                if i == 1
                    if nargin < 2 || isempty(zGuess)
                        zGuess = obj.generateInitialGuess();
                    end
                else
                    timeOld = linspace(obj.t0, obj.tF, obj.nGrid(i-1));
                    zGuess = [solution(i-1).z.state; solution(i-1).z.control];
                    zGuess = interp1(timeOld, zGuess', time)';
                end
                
                % Set up the problem
                fun = @(z)( obj.objective(time, z(1:obj.nx,:), z(obj.nx+1:end,:)) );
                A = []; b = []; Aeq = []; beq = [];
                lb = repmat([obj.xLow; obj.uLow], 1, obj.nGrid(i));
                ub = repmat([obj.xUpp; obj.uUpp], 1, obj.nGrid(i));
                nonlcon = @(z)( obj.constraints(time, z(1:obj.nx,:), z(obj.nx+1:end,:)) );
                
                [z, fval, exitflag, output] = fmincon(fun,zGuess,A,b,Aeq,beq,lb,ub,nonlcon,obj.solverOptions{i});
                solution(i).nGrid = obj.nGrid(i);
                solution(i).z = struct();
                solution(i).z.time = time;
                solution(i).z.state = z(1:obj.nx,:);
                solution(i).z.control = z(obj.nx+1:end,:);
                solution(i).z.derivatives = obj.dynamics(time, solution(i).z.state, solution(i).z.control, obj.parameters);
                solution(i).fval = fval;
                solution(i).exitflag = exitflag;
                solution(i).output = output;
                
                % Check constraints if a check function is provided
                if ~isempty(obj.constraintsCheck)
                    solution(i).violations = obj.constraintsCheck(...
                        time, ...
                        solution(i).z.state, ...
                        solution(i).z.control, ...
                        obj.parameters);
                end
            end
        end
        
        function [t0, tF] = getTimeBounds(obj)
            % Get time bounds
            t0 = obj.t0;
            tF = obj.tF;
        end
        
        function [x0, xF] = getBoundaryConditions(obj)
            % Get boundary conditions
            x0 = obj.x0;
            xF = obj.xF;
        end

        function parameters = getParameters(obj)
            % Get problem parameters
            parameters = obj.parameters;
        end

        function nGrid = getGridSize(obj)
            % Get grid size
            nGrid = obj.nGrid;
        end
    end
end