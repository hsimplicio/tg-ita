t = readmatrix('psopt/tt0.dat');
x = readmatrix('psopt/xt0.dat');
u = readmatrix('psopt/ut0.dat');
XY = readmatrix('psopt/XY.dat');

figure;
plot(t, x(1,:));

solution = load('solution-base-t45.mat');
z = struct();
z.time = t;
z.state = x;
z.control = u;

addpath('../evtol');
plotEvtolResults('Base', z);
