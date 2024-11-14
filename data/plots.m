clc; clear; close all;

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

% Plot MATLAB solution
s = load('../evtol/results/solution-base-t45.mat');
w = struct();
w.time = s.solution(end).z.time;
w.state = s.solution(end).z.state;
w.control = s.solution(end).z.control;

% Create comparison plots
figure('Position', [100, 100, 1200, 1000]);

% State comparison
subplot(3, 2, 1)
plot(z.time, z.state(1,:), 'b-', 'LineWidth', 2, 'DisplayName', 'PSOPT x');
hold on;
plot(w.time, w.state(1,:), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB x');
plot(z.time, z.state(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'PSOPT y');
plot(w.time, w.state(2,:), 'm--', 'LineWidth', 2, 'DisplayName', 'MATLAB y');
grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Posição [m]', 'FontSize', 12);
title('Comparação de Posição', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 12);

% Control comparison
subplot(3, 2, 2)
plot(z.time, z.control, 'b-', 'LineWidth', 2, 'DisplayName', 'PSOPT');
hold on;
plot(w.time, w.control, 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Controle', 'FontSize', 12);
title('Comparação de Controle', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 12);
% Trajectory comparison
subplot(3, 2, 3)
plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'PSOPT');
hold on;
plot(w.state(1,:), w.state(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
grid on;
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Comparação de Trajetória', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 12);

% Phase portrait
subplot(3, 2, 4)
plot(z.state(3,:), z.state(4,:), 'b-', 'LineWidth', 2, 'DisplayName', 'PSOPT');
hold on;
plot(w.state(3,:), w.state(4,:), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
grid on;
xlabel('vx [m/s]', 'FontSize', 12);
ylabel('vy [m/s]', 'FontSize', 12);
title('Comparação de Velocidade', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 12);

% Energy comparison
subplot(3, 2, [5 6])
plot(z.time, z.state(5,:), 'b-', 'LineWidth', 2, 'DisplayName', 'PSOPT');
hold on;
plot(w.time, w.state(5,:), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Energia [J]', 'FontSize', 12);
title('Comparação de Consumo de Energia', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 12);

% Save the figure
savefig('comparison_plot.fig');
saveas(gcf, 'comparison_plot.png');
saveas(gcf, 'comparison_plot.svg');


