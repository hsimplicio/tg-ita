%%
clc; clear;

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
p.aircraft.wing.CD_p = 0.0437;  % wing parasite drag coefficient
p.aircraft.wing.AR = 20.0;  % wing aspect ratio
p.aircraft.wing.CL_zero = 0.28;  % wing lift coefficient at zero angle of attack
p.aircraft.wing.CL_alpha = 4.00;  % wing lift coefficient slope
p.aircraft.wing.alpha_fus = 0 * (pi / 180);  % wing angle of attack
p.aircraft.wing.sigmoid.M = 4.0;  % wing lift coefficient slope factor
p.aircraft.wing.sigmoid.alpha_0 = 20 * (pi / 180);  % wing lift coefficient zero-lift angle

% Trajectory parameters
duration = 45;  % s

%% Set up function handles

% Objective functions
phi = @(x0, xF, t0, tF)( bnd_obj(x0,xF,t0,tF,[]) );  % boundary objective (φ(·) - Mayer Term)
L = @(t,x,u)( path_obj(t,x,u) );  % path objective (L(·) - integrand of Lagrange Term)

obj_fun = @(t,x,u)( objective(t,x,u,phi,L) );  % objective function (J(·))

% Constraint functions
f = @(t,x,u)( dyn_evtol(x,u,p) );  % dynamics function (f(·))
zeta = @(dt,x,f)( defects(dt,x,f) );  % defect constraints (ζ(·))
g = @(t,x,u)( path_cst(t,x,u) );  % path constraint (g(·))
h = @(x0, xF, t0, tF)( bnd_cst(x0,xF,t0,tF) );  % boundary constraint (h(·))

cst_fun = @(t,x,u)( constraints(t,x,u,f,zeta,g,h) );  % constraint function


%% Set up problem bounds
% initial_time.low = 0;
% initial_time.upp = 0;
% final_time.low = duration;
% final_time.upp = duration;

% initial_state.low = x0;
% initial_state.upp = x0;
% final_state.low = xF;
% final_state.upp = xF;

state.low = [0; -10; 0; -5; 0];
state.upp = [1000; 110; 35; 6; 1e8];

control.low = [0; 0];
control.upp = [1800; 2600];


%% Solver options
n_grid = [50,80,150,190,250];  % number of discretization points
% n_grid = [100,150];

options = optimoptions('fmincon');
options.Display = 'iter';
options.MaxFunEvals = 1e5;


%% Initial guess at trajectory
% Interpolate the guess at the grid-points for transcription:
time = linspace(0, duration, n_grid);
x0_guess = [0; 0; 0; 0; 0];
xF_guess = [0; 0; 0; 0; Inf];
u0_guess = [0; p.aircraft.m * p.environment.g];
uF_guess = [0; p.aircraft.m * p.environment.g];

dx0_guess = dyn_evtol(x0_guess, u0_guess, p);

xF_guess(5) = x0_guess(5) + dx0_guess(5) * duration;

x_guess = interp1([0, duration]', [x0_guess, xF_guess]', time')';
u_guess = interp1([0, duration]', [u0_guess, uF_guess]', time')';

z_guess = [x_guess; u_guess];

figure;
subplot(4,2,[1,2]);
plot(time, x_guess(5,:));
title('Battery State');
xlabel('t (s)');
ylabel('E (J)');
hold on;
plot([0, duration], [x0_guess(5), xF_guess(5)], 'ro');
hold off;

subplot(4,2,3);
plot(time, x_guess(1,:));
title('Horizontal Position');
xlabel('$t$ (s)');
ylabel('$s_x$ (m)');
hold on;
plot(time(1), x0_guess(1), 'ro');
plot(time(end), xF_guess(1), 'rx');
hold off;

subplot(4,2,4);
plot(time, x_guess(2,:));
title('Vertical Position');
xlabel('$t$ (s)');
ylabel('$s_y$ (m)');
hold on;
plot(time(1), x0_guess(2), 'ro');
plot(time(end), xF_guess(2), 'rx');
hold off;

subplot(4,2,5);
plot(time, x_guess(3,:));
title('Horizontal Speed');
xlabel('t (s)');
ylabel('$v_x$ (m/s)');
hold on;
plot([0, duration], [x0_guess(3), xF_guess(3)], 'ro');
hold off;

subplot(4,2,6);
plot(time, x_guess(4,:));
title('Vertical Speed');
xlabel('t (s)');
ylabel('$v_y$ (m/s)');
hold on;
plot([0, duration], [x0_guess(4), xF_guess(4)], 'ro');
hold off;

subplot(4,2,7);
plot(time, u_guess(1,:));
title('Horizontal Thrust');
xlabel('t (s)');
ylabel('$T_x$ (N)');
hold on;
plot([0, duration], [u0_guess(1), uF_guess(1)], 'ro');
hold off;

subplot(4,2,8);
plot(time, u_guess(2,:));
title('Vertical Thrust');
xlabel('t (s)');
ylabel('$T_y$ (N)');
hold on;
plot([0, duration], [u0_guess(2), uF_guess(2)], 'ro');
hold off;


%% Solve!
n_iter = length(n_grid);
solution(n_iter) = struct();
for i = 1:n_iter
    disp(['Iteration ', num2str(i), ' of ', num2str(n_iter)]);
    if i > 1
        time_old = time;
        time = linspace(0, duration, n_grid(i));
        z_guess = solution(i-1).z;
        z_guess = interp1(time_old, z_guess', time)';
    end

    % Set up the problem
fun = @(z)( obj_fun(time, z(1:5,:), z(6:7,:)) );
A = []; b = []; Aeq = []; beq = [];
    lb = repmat([state.low; control.low], 1, n_grid(i));
    ub = repmat([state.upp; control.upp], 1, n_grid(i));
nonlcon = @(z)( cst_fun(time, z(1:5,:), z(6:7,:)) );

    [z, fval, exitflag, output] = fmincon(fun,z_guess,A,b,Aeq,beq,lb,ub,nonlcon,options);
    solution(i).z = z;
    solution(i).fval = fval;
    solution(i).exitflag = exitflag;
    solution(i).output = output;
end

z = solution(end).z;
save('solution.mat', 'z');

%%
% Extract the results
x = z(1:5,:);
u = z(6:7,:);

dx = dyn_evtol(x, u, p);

%%
% Plot the results
set(groot, 'defaultTextInterpreter', 'latex');

figure;
subplot(4,2,[1,2]);
plot(time, dx(5,:));
title('Battery Rate');
xlabel('t (s)');
ylabel('$\dot{E}$ (W)');

subplot(4,2,3);
plot(time, x(1,:));
title('Horizontal Position');
xlabel('$t$ (s)');
ylabel('$s_x$ (m)');

subplot(4,2,4);
plot(time, x(2,:));
title('Vertical Position');
xlabel('$t$ (s)');
ylabel('$s_y$ (m)');

subplot(4,2,5);
plot(time, x(3,:));
title('Horizontal Speed');
xlabel('t (s)');
ylabel('$v_x$ (m/s)');

subplot(4,2,6);
plot(time, x(4,:));
title('Vertical Speed');
xlabel('t (s)');
ylabel('$v_y$ (m/s)');

subplot(4,2,7);
plot(time, u(1,:));
title('Horizontal Thrust');
xlabel('t (s)');
ylabel('$T_x$ (N)');

subplot(4,2,8);
plot(time, u(2,:));
title('Vertical Thrust');
xlabel('t (s)');
ylabel('$T_y$ (N)');

figure;
plot(time, x(5,:), 'DisplayName', 'Battery State');
hold on;
plot(time, dx(5,:), 'DisplayName', 'Battery Rate');
hold off;
title('Battery');
xlabel('t (s)');
ylabel('Battery State / Rate');
legend show;
