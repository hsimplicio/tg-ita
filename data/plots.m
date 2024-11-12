addpath('..');

% Plot PSOPT solution
t = readmatrix('../data/psopt/tt0.dat');
x = readmatrix('../data/psopt/xt0.dat');
u = readmatrix('../data/psopt/ut0.dat');
XY = readmatrix('../data/psopt/XY.dat');

z = struct();
z.time = t;
z.state = x;
z.control = u;

plotEvtolResults('PSOPT', z);

% Plot MATLAB solution
solution = load('solution-base-t45.mat');
z = struct();
z.time = linspace(0, 45, size(solution.z, 2));
z.state = solution.z(1:5,:);
z.control = solution.z(6:end,:);

plotEvtolResults('Base', z);
