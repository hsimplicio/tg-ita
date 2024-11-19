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
% figure('Position', [100, 100, 1200, 1000]);
figure('Name', 'Comparison', 'Position', [100, 100, 1200, 1000]);

% State comparison
subplot(2, 2, 1)
plot(z.time, z.state(1,:), 'b-', 'LineWidth', 2, 'DisplayName', 'x_{ref}');
hold on;
plot(w.time, w.state(1,:), 'r--', 'LineWidth', 2, 'DisplayName', 'x');
plot(z.time, z.state(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'y_{ref}');
plot(w.time, w.state(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'y');
grid on;
xlabel('t [s]', 'FontSize', 18);
ylabel('Posição [m]', 'FontSize', 18);
title('Comparação de Posição', 'FontSize', 18);
legend('Location', 'eastoutside');
set(gca, 'FontSize', 18);

% Control comparison
subplot(2, 2, 2)
plot(z.time, z.control(1,:), 'b-', 'LineWidth', 2, 'DisplayName', 'T_{x,ref}');
hold on;
plot(z.time, z.control(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'T_{y,ref}');
plot(w.time, w.control(1,:), 'r--', 'LineWidth', 2, 'DisplayName', 'T_{x}');
plot(w.time, w.control(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'T_{y}');
grid on;
xlabel('t [s]', 'FontSize', 18);
ylabel('Controle', 'FontSize', 18);
title('Comparação de Controle', 'FontSize', 18);
legend('Location', 'eastoutside');
set(gca, 'FontSize', 18);

% Trajectory comparison
subplot(2, 2, 3)
plot(z.state(1,:), z.state(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'referência');
hold on;
plot(w.state(1,:), w.state(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'trajetória');
grid on;
xlabel('x [m]', 'FontSize', 18);
ylabel('y [m]', 'FontSize', 18);
title('Comparação de Trajetória', 'FontSize', 18);
legend('Location', 'eastoutside');
set(gca, 'FontSize', 18);

% Phase portrait
subplot(2, 2, 4)
plot(z.time, z.state(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'v_{x,ref}');
hold on;
plot(z.time, z.state(4,:), 'b-', 'LineWidth', 2, 'DisplayName', 'v_{y,ref}');
plot(w.time, w.state(3,:), 'r--', 'LineWidth', 2, 'DisplayName', 'v_{x}');
plot(w.time, w.state(4,:), 'r--', 'LineWidth', 2, 'DisplayName', 'v_{y}');
grid on;
xlabel('vx [m/s]', 'FontSize', 18);
ylabel('vy [m/s]', 'FontSize', 18);
title('Comparação de Velocidade', 'FontSize', 18);
legend('Location', 'eastoutside');
set(gca, 'FontSize', 18);

% Save the figure
savefig('comparison_plot.fig');
saveas(gcf, 'comparison_plot.png');
saveas(gcf, 'comparison_plot.svg');


% Energy comparison
figure('Name', 'Energy comparison');
% subplot(3, 2, [5 6])
plot(z.time, z.state(5,:), 'b-', 'LineWidth', 2, 'DisplayName', 'referência');
hold on;
plot(w.time, w.state(5,:), 'r--', 'LineWidth', 2, 'DisplayName', 'consumo');
grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Energia [J]', 'FontSize', 12);
title('Comparação de Consumo de Energia', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 12);

% Save the figure
savefig('energy_comparison_plot.fig');
saveas(gcf, 'energy_comparison_plot.png');
saveas(gcf, 'energy_comparison_plot.svg');


