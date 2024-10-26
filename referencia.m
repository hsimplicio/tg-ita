%% Declare key structures
algorithm = Algorithm();
problem = Problem();

%% Register problem name
problem.name = "OptimClimb";
problem.outfilename = "OptimClimb.txt";

%% Define problem level constants and do level 1 setup
problem.nphases = 1;
problem.nlinkages = 0;
psopt_level1_setup(problem); % ????

%% Define phase related information & do level 2 setup
problem.phases(1).nstates = 5;
problem.phases(1).ncontrols = 2;
problem.phases(1).nevents = 9;
problem.phases(1).npath = 0;
problem.phases(1).nodes = [50 80 150 190 250];
psopt_level2_setup(problem, algorithm);

%% Enter problem bounds information
xL = 0.0;
yL = -10.0;
vxL = 0.0;
vyL = -5.0;

xU = 1000.0;
yU = 110.0;
vxU = 35.0;
vyU = 6.0;

TxL = 0.0;
TyL = 0.0;
TxU = 1800.0;
TyU = 2600.0;

x0 = 0.0;
y0 = 0.0;
vx0 = 0.0;
vy0 = 0.0;
xf = 1000.0;
yf = 100.0;
vxf = 25.0;
vyf = 0.0;
E0 = 0.0;

problem.phases(1).bounds.lower.states(0) = xL;
problem.phases(1).bounds.lower.states(1) = yL;
problem.phases(1).bounds.lower.states(2) = vxL;
problem.phases(1).bounds.lower.states(3) = vyL;
problem.phases(1).bounds.lower.states(4) = 0.0;

problem.phases(1).bounds.upper.states(0) = xU;
problem.phases(1).bounds.upper.states(1) = yU;
problem.phases(1).bounds.upper.states(2) = vxU;
problem.phases(1).bounds.upper.states(3) = vyU;
problem.phases(1).bounds.upper.states(4) = 1.e+8;

problem.phases(1).bounds.lower.controls(0) = TxL;
problem.phases(1).bounds.lower.controls(1) = TyL;
problem.phases(1).bounds.upper.controls(0) = TxU;
problem.phases(1).bounds.upper.controls(1) = TyU;

problem.phases(1).bounds.lower.events(0) = x0;
problem.phases(1).bounds.lower.events(1) = y0;
problem.phases(1).bounds.lower.events(2) = vx0;
problem.phases(1).bounds.lower.events(3) = vy0;
problem.phases(1).bounds.lower.events(4) = xf;
problem.phases(1).bounds.lower.events(5) = yf;
problem.phases(1).bounds.lower.events(6) = vxf;
problem.phases(1).bounds.lower.events(7) = vyf;
problem.phases(1).bounds.lower.events(8) = E0;

problem.phases(1).bounds.upper.events(0) = x0;
problem.phases(1).bounds.upper.events(1) = y0;
problem.phases(1).bounds.upper.events(2) = vx0;
problem.phases(1).bounds.upper.events(3) = vy0;
problem.phases(1).bounds.upper.events(4) = xf;
problem.phases(1).bounds.upper.events(5) = yf;
problem.phases(1).bounds.upper.events(6) = vxf;
problem.phases(1).bounds.upper.events(7) = vyf;
problem.phases(1).bounds.upper.events(8) = E0;

problem.phases(1).bounds.lower.StartTime = 0.0;
problem.phases(1).bounds.upper.StartTime = 0.0;

problem.phases(1).bounds.lower.EndTime = 45.0;
problem.phases(1).bounds.upper.EndTime = 135.0;

%% Register problem functions
problem.integrand_cost = @integrand_cost;
problem.endpoint_cost = @endpoint_cost;
problem.dae = @dae;
problem.events = @events;
% problem.linkages = @linkages;

%% Define and register initial guess
nnodes = problem.phases(1).nodes(0);
ncontrols = problem.phases(1).ncontrols;
nstates = problem.phases(1).nstates;

% TODO: Definir MatrixXd?
x_guess = zeros(nstates, nnodes);

x_guess.row(0) = x0 * ones(1, nnodes);
x_guess.row(1) = y0 * ones(1, nnodes);
x_guess.row(2) = vx0 * ones(1, nnodes);
x_guess.row(3) = vy0 * ones(1, nnodes);
x_guess.row(4) = E0 * ones(1, nnodes);

problem.phases(1).guess.controls = zeros(ncontrols, nnodes);
problem.phases(1).guess.states = x_guess;
problem.phases(1).guess.time = linspace(0.0, 45.0, nnodes);

%% Enter algorithm options
algorithm.nlp_iter_max = 5000;
algorithm.nlp_tolerance = 1.e-6;
algorithm.nlp_method = "IPOPT";
algorithm.scaling = "automatic";
algorithm.derivatives = "automatic";
% algorithm.mesh_refinement = "automatic";
algorithm.collocation_method = "trapezoidal";
% algorithm.defect_scaling = "jacobian-based";
% algorithm.ode_tolerance = 1.e-5;

%% Call the tool to solve the problem
solution = solve(problem, algorithm);

%% Extract solution data
x = solution.get_states_in_phase(1);
u = solution.get_controls_in_phase(1);
t = solution.get_time_in_phase(1);
lambda = solution.get_dual_costates_in_phase(1);
H = solution.get_dual_hamiltonian_in_phase(1);
stx = x.row(1);
sty = x.row(2);
stvx = x.row(3);
stvy = x.row(4);
stEn = x.row(5);

%% Save solution data to files
save("OptimClimb_solution.mat", "x", "u", "t", "lambda", "H");

%% Plot results
plot(t, stx);
plot(t, sty);
plot(t, stvx);
plot(t, stvy);
plot(t, x);
plot(t, u);
plot(t, stEn);