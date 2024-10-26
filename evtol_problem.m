%%
clc; clear;

%%
% for test only
x = [
    0, 0, 0, 0, 0; % sx
    0, 0, 0, 0, 0; % sy
    10, 20, 30, 40, 50; % vx
    5, 15, 25, 35, 45; % vy
    100, 200, 300, 400, 500 % E
];

u = [
    100, 200, 300, 400, 500; % Tx
    50, 150, 250, 350, 450 % Ty
];

%%
% Dynamics parameters
p.environment.g = 9.78;  % m/s^2 gravity
p.environment.rho = 1.2;  % kg/m^3 air density

p.aircraft.m = 240.0;  % kg mass
p.aircraft.A = 1.6;  % m^2 rotor area
p.aircraft.chi = 1.0;  % rotor-to-rotor interference factor

p.aircraft.fuselage.CD = 1.0;  % fuselage drag coefficient
p.aircraft.fuselage.Sf = 2.11;  % fuselage cross-sectional area
p.aircraft.fuselage.St = 1.47;  % fuselage top area

p.aircraft.wing.S = 4.0;  % wing area
p.aircraft.wing.e = 0.9;  % wing span efficiency factor
p.aircraft.wing.CD_p = 0.0437;  % wing profile drag coefficient
p.aircraft.wing.AR = 20.0;  % wing aspect ratio
p.aircraft.wing.CL_zero = 0.28;  % wing lift coefficient at zero angle of attack
p.aircraft.wing.CL_alpha = 4.00;  % wing lift coefficient slope
p.aircraft.wing.alpha_fus = 0 * (pi / 180);  % wing angle of attack
p.aircraft.wing.sigmoid.n_grid = 4.0;  % wing lift coefficient slope factor
p.aircraft.wing.sigmoid.alpha_0 = 20 * (pi / 180);  % wing lift coefficient zero-lift angle

% Trajectory parameters
duration = 45;  % s

% Initial State:
% [sx; sy; vx; vy; E]
x0 = zeros(5, 1);  % initial state
xF = [1000; 100; 25; 0; 0];  % final state

% Initial Control:
% [Tx; Ty]
u0 = zeros(2, 1);  % initial control
uF = zeros(2, 1);  % final control


%% Set up function handles
dynamics = @(t,x,u)( dyn_evtol(x,u,p) );

% path_obj = @(t,x,u)( path_obj(t,x,u) );  % path objective (L(·) - integrand of Lagrange Term)
bnd_obj = @(x0, xF, t0, tF)( bnd_obj(x0, xF, t0, tF, []) );  % boundary objective (φ(·) - Mayer Term)

obj_fun = @(t,x,u)( objective(t,x,u,[],bnd_obj) );  % objective function (J(·))

f = @(t,x,u)( defects(t,x,u,dynamics) );  % defect constraints (f(·))
g = @(x0, xF, t0, tF)( bnd_cst(x0, xF, t0, tF, []) );  % boundary constraint (g(·))
h = @(t,x,u)( path_cst(t,x,u) );  % path constraint (h(·))

cst_fun = @(t,x,u)( constraints(t,x,u,f,g,h) );  % constraint function (g(·) and h(·))


%% Set up problem bounds
initial_time.low = 0;
initial_time.upp = 0;
final_time.low = duration;
final_time.upp = duration;

initial_state.low = x0;
initial_state.upp = x0;
final_state.low = xF;
final_state.upp = xF;

control.low = [0; 0];
control.upp = [1800; 2600];



%% Solver options
n_grid = 40;  % number of discretization points
nlpOpt = optimset('Display','iter','MaxFunEvals',1e5);



%% Initial guess at trajectory
% Interpolate the guess at the grid-points for transcription:
time = linspace(0, duration, n_grid);
z_guess0 = repmat([x0; u0], 1, n_grid);

z_guess = [time; z_guess0];


%% Solve!



