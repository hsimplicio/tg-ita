classdef TrajectoryProblem < handle
    % Class to define a trajectory optimization problem
    
    properties (SetAccess = private)
        % Problem dimensions
        nx  % Number of states
        nu  % Number of controls
        
        % Time boundary conditions
        t0 {mustBeNumeric}  % Initial time
        tF {mustBeNumeric}  % Final time
        timeSpan = {}  % Cell array of time spans

        % Boundary conditions
        x0  % Initial state
        xF  % Final state

        % Boundary conditions struct
        boundaryConditions

        % Time bounds
        t0Low {mustBeNumeric}  % Lower bound for initial time
        t0Upp {mustBeNumeric}  % Upper bound for initial time
        tFLow {mustBeNumeric}  % Lower bound for final time
        tFUpp {mustBeNumeric}  % Upper bound for final time
        
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
        
        % Scaling factors
        stateScaling
        controlScaling
        timeScaling
        scaling % Struct containing stateScaling, controlScaling, and timeScaling
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

            % Initialize state and control bounds
            obj.xLow = -1e8*ones(nx, 1);
            obj.xUpp = 1e8*ones(nx, 1);
            obj.uLow = -1e8*ones(nu, 1);
            obj.uUpp = 1e8*ones(nu, 1);

            % Initialize time bounds
            obj.t0Low = -1e8;
            obj.t0Upp = 1e8;
            obj.tFLow = -1e8;
            obj.tFUpp = 1e8;
            
            % Default time boundary conditions
            obj.t0 = 0;
            obj.tF = 1;
            obj.timeSpan{1} = [obj.t0, obj.tF];

            % Initialize boundary conditions with empty arrays
            obj.x0 = zeros(nx, 1);
            obj.xF = zeros(nx, 1);
            
            % Initialize scaling factors to 1
            obj.stateScaling = ones(nx, 1);
            obj.controlScaling = ones(nu, 1);
            obj.timeScaling = 1;
            obj.scaling = struct('stateScaling', obj.stateScaling, ...
                'controlScaling', obj.controlScaling, ...
                    'timeScaling', obj.timeScaling);
                    
            % Initialize boundary conditions struct
            obj.boundaryConditions = struct();
            obj.boundaryConditions.t0 = obj.t0 / obj.timeScaling;
            obj.boundaryConditions.tF = obj.tF / obj.timeScaling;
            obj.boundaryConditions.x0 = obj.x0 / obj.stateScaling;
            obj.boundaryConditions.xF = obj.xF / obj.stateScaling;
        end

        function setBoundaryConditions(obj, x0, xF)
            % Set boundary conditions
            validateattributes(x0, {'numeric'}, {'vector', 'numel', obj.nx});
            validateattributes(xF, {'numeric'}, {'vector', 'numel', obj.nx});
            obj.x0 = x0(:);  % Ensure column vector
            obj.xF = xF(:);
            obj.boundaryConditions.x0 = x0;
            obj.boundaryConditions.xF = xF;
        end
        
        function setTimeBoundaryConditions(obj, t0, tF)
            % Set time bounds
            validateattributes(t0, {'numeric'}, {'scalar'});
            validateattributes(tF, {'numeric'}, {'scalar', '>', t0});
            obj.t0 = t0;
            obj.tF = tF;
            obj.timeSpan{1} = [obj.t0, obj.tF];
            obj.boundaryConditions.t0 = t0;
            obj.boundaryConditions.tF = tF;
        end

        function setTimeBounds(obj, t0Low, t0Upp, tFLow, tFUpp)
            % Set bounds for initial and final times
            validateattributes(t0Low, {'numeric'}, {'scalar'});
            validateattributes(t0Upp, {'numeric'}, {'scalar'});
            validateattributes(tFLow, {'numeric'}, {'scalar'});
            validateattributes(tFUpp, {'numeric'}, {'scalar'});
            
            assert(t0Low <= t0Upp, 'Lower bound for t0 must be <= upper bound');
            assert(tFLow <= tFUpp, 'Lower bound for tF must be <= upper bound');
            assert(t0Upp <= tFLow, 'Upper bound for t0 must be <= lower bound for tF');
            
            obj.t0Low = t0Low;
            obj.t0Upp = t0Upp;
            obj.tFLow = tFLow;
            obj.tFUpp = tFUpp;
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

        function setScaling(obj, type, factors)
            % Set scaling factors for variables
            % type: 'state', 'control', or 'time'
            % factors: vector of scaling factors
            
            switch lower(type)
                case 'state'
                    validateattributes(factors, {'numeric'}, {'vector', 'positive', 'numel', obj.nx});
                    obj.stateScaling = factors(:);
                    obj.scaling.stateScaling = factors(:);
                    obj.boundaryConditions.x0 = obj.x0 / obj.stateScaling;
                    obj.boundaryConditions.xF = obj.xF / obj.stateScaling;
                case 'control'
                    validateattributes(factors, {'numeric'}, {'vector', 'positive', 'numel', obj.nu});
                    obj.controlScaling = factors(:);
                    obj.scaling.controlScaling = factors(:);
                case 'time'
                    validateattributes(factors, {'numeric'}, {'scalar', 'positive'});
                    obj.timeScaling = factors;
                    obj.scaling.timeScaling = factors;
                    obj.boundaryConditions.t0 = obj.t0 / obj.timeScaling;
                    obj.boundaryConditions.tF = obj.tF / obj.timeScaling;
                otherwise
                    error('Invalid scaling type. Must be ''state'', ''control'', or ''time''');
            end
        end
        
        function setDynamics(obj, hDynamics)
            % Set dynamics function
            validateattributes(hDynamics, {'function_handle'}, {});
            % % Test the function with dummy inputs
            % time_test = zeros(1,1);
            % x_test = zeros(obj.nx, 1);
            % u_test = zeros(obj.nu, 1);
            % try
            %     dx = hDynamics(time_test, x_test, u_test, obj.parameters);
            %     validateattributes(dx, {'numeric'}, {'vector', 'numel', obj.nx});
            % catch ME
            %     error('Invalid dynamics function: %s', ME.message);
            % end
            obj.dynamics = @(time, state, control) hDynamics(time, state, control, obj.parameters);
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
                    validateattributes(val, {'numeric'}, {'size', [1, numel(time_test)]});
                catch ME
                    error('Invalid path objective function: %s', ME.message);
                end
                obj.pathObjective = @(time, state, control) hPathObjective(time, state, control, obj.parameters);
            else
                obj.pathObjective = [];
            end
            
            % Create combined objective function
            obj.objective = @(z, packInfo) evaluateObjective(...
                z, packInfo, ...
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

                % Scale boundary conditions
                bndConditions = struct();
                if isfield(obj.boundaryConditions, 't0')
                    bndConditions.t0 = obj.boundaryConditions.t0 / obj.timeScaling;
                end
                if isfield(obj.boundaryConditions, 'tF')
                    bndConditions.tF = obj.boundaryConditions.tF / obj.timeScaling;
                end
                if isfield(obj.boundaryConditions, 'x0')
                    bndConditions.x0 = obj.boundaryConditions.x0;
                end
                if isfield(obj.boundaryConditions, 'xF')
                    bndConditions.xF = obj.boundaryConditions.xF;
                end
                obj.boundaryConstraints = @(x0, xF, t0, tF) hBoundaryConstraints(x0, xF, t0, tF, bndConditions);
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
            obj.constraints = @(z, packInfo) evaluateConstraints(...
                z, packInfo, ...
                obj.dynamics, ...
                @computeDefects, ...
                obj.pathConstraints, ...
                obj.boundaryConstraints);
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
        
        function [state, control] = generateInitialGuess(obj)
            % Generate initial guess with reasonable values for free final states
            
            % Validate boundary conditions exist
            if isempty(obj.x0) || isempty(obj.xF)
                error('TrajectoryProblem:NoBoundaryConditions', ...
                    'Either pass initial guess or set boundary conditions to generate a linear interpolation initial guess.');
            end

            timeGrid = linspace(obj.t0, obj.tF, obj.nGrid(1));
            
            % Initialize state array
            state = zeros(obj.nx, obj.nGrid(1));
            
            % For each state variable
            for i = 1:obj.nx
                if abs(obj.xF(i)) > 1e6  % Detect "free" final states
                    % Use a reasonable final value instead of the large number
                    if obj.xF(i) > 0
                        finalVal = max(obj.x0(i), obj.xUpp(i)/2);  % Use half of upper bound
                    else
                        finalVal = min(obj.x0(i), obj.xLow(i)/2);  % Use half of lower bound
                    end
                    state(i,:) = interp1([obj.t0, obj.tF], [obj.x0(i), finalVal], timeGrid);
                else
                    % Normal linear interpolation for fixed final states
                    state(i,:) = interp1([obj.t0, obj.tF], [obj.x0(i), obj.xF(i)], timeGrid);
                end
            end
            
            % Initialize controls to zeros
            control = zeros(obj.nu, obj.nGrid(1));
        end
        
        function solution = solveWithTrapezoidalCollocation(obj, guess)
            % Solve the trajectory optimization problem with trapezoidal collocation
            if ~obj.validate()
                error('TrajectoryProblem:InvalidProblem', ...
                    'Problem is not completely defined');
            end
            
            nIter = length(obj.nGrid);
            solution(nIter) = struct();
            
            for i = 1:nIter
                disp(['Iteration ', num2str(i), ' of ', num2str(nIter)]);

                % Generate or interpolate initial guess
                if i == 1
                    timeGrid = linspace(obj.t0, obj.tF, obj.nGrid(1));
                    if nargin < 2 || isempty(guess)
                        [stateGuess, controlGuess] = obj.generateInitialGuess();
                        [zGuess, packInfo] = packZ([obj.t0, obj.tF], stateGuess, controlGuess, obj.scaling);
                    else
                        [zGuess, packInfo] = packZ([obj.t0, obj.tF], guess(1:obj.nx,:), guess(obj.nx+1:end,:), obj.scaling);
                    end
                else
                    % Validate time span
                    assert(all(isfinite(obj.timeSpan{i-1})), 'Non-finite values in previous time span');
                    assert(all(isfinite(obj.timeSpan{i})), 'Non-finite values in current time span');
                    assert(obj.timeSpan{i-1}(2) >= obj.timeSpan{i-1}(1), 'Invalid previous time span');
                    assert(obj.timeSpan{i}(2) >= obj.timeSpan{i}(1), 'Invalid current time span');

                    timeGrid = linspace(obj.timeSpan{i}(1), obj.timeSpan{i}(2), obj.nGrid(i));
                    timeOld = linspace(obj.timeSpan{i-1}(1), obj.timeSpan{i-1}(2), obj.nGrid(i-1));
                    
                    % Validate previous solution
                    assert(all(isfinite(solution(i-1).z.state(:))), 'Non-finite values in previous state');
                    assert(all(isfinite(solution(i-1).z.control(:))), 'Non-finite values in previous control');
                    
                    % Perform interpolation with validation
                    stateGuess = zeros(obj.nx, obj.nGrid(i));
                    controlGuess = zeros(obj.nu, obj.nGrid(i));
                    
                    % Interpolate each state and control separately to maintain dimensions
                    for j = 1:obj.nx
                        [stateGuess(j,:), ~] = spline2(timeOld, solution(i-1).z.state(j,:), solution(i-1).z.derivatives(j,:), timeGrid);
                    end
                    for j = 1:obj.nu
                        controlGuess(j,:) = interp1(timeOld, solution(i-1).z.control(j,:), timeGrid, 'linear', 'extrap');
                    end
                    
                    % Validate interpolation results
                    assert(all(isfinite(stateGuess(:))), 'Non-finite values in interpolated state');
                    assert(all(isfinite(controlGuess(:))), 'Non-finite values in interpolated control');
                    
                    % Pack with validation
                    [zGuess, packInfo] = packZ(obj.timeSpan{i}, stateGuess, controlGuess, obj.scaling);
                    assert(all(isfinite(zGuess)), 'Non-finite values in packed guess');
                end
                
                % Set up the problem
                fun = @(z)( obj.objective(z, packInfo) );
                A = []; b = []; Aeq = []; beq = [];
                lb = [
                    obj.t0Low / obj.scaling.timeScaling;
                    obj.tFLow / obj.scaling.timeScaling;
                    reshape(repmat(obj.xLow ./ obj.scaling.stateScaling, 1, obj.nGrid(i)), [], 1);
                    reshape(repmat(obj.uLow ./ obj.scaling.controlScaling, 1, obj.nGrid(i)), [], 1)
                ];
                ub = [
                    obj.t0Upp / obj.scaling.timeScaling;
                    obj.tFUpp / obj.scaling.timeScaling;
                    reshape(repmat(obj.xUpp ./ obj.scaling.stateScaling, 1, obj.nGrid(i)), [], 1);
                    reshape(repmat(obj.uUpp ./ obj.scaling.controlScaling, 1, obj.nGrid(i)), [], 1)
                ];
                nonlcon = @(z)( obj.constraints(z, packInfo) );

                % Solve scaled problem
                [z_scaled, fval, exitflag, output] = fmincon(fun, zGuess, ...
                    A, b, Aeq, beq, lb, ub, nonlcon, obj.solverOptions{i});

                % Unpack solution with scaling up
                [time, state, control] = unpackZ(z_scaled, packInfo, true);

                % Set time span for next iteration
                obj.timeSpan{i+1} = [time(1), time(end)];
                
                solution(i).nGrid = obj.nGrid(i);
                solution(i).z = struct();
                solution(i).z.timeSpan = obj.timeSpan{i+1};
                solution(i).z.time = timeGrid;
                solution(i).z.state = state;
                solution(i).z.control = control;
                solution(i).z.derivatives = obj.dynamics(timeGrid, state, control);
                solution(i).fval = fval;
                solution(i).exitflag = exitflag;
                solution(i).output = output;
                
                % Check constraints if a check function is provided
                if ~isempty(obj.constraintsCheck)
                    solution(i).violations = obj.constraintsCheck(...
                        timeGrid, ...
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