clear; clc; close all;

% Test functions: 
% State 1: f1(t) = sin(t),    df1/dt = cos(t)
% State 2: f2(t) = t^2,       df2/dt = 2t
% State 3: f3(t) = exp(-t),   df3/dt = -exp(-t)

f = @(t) [sin(t); t.^2; exp(-t)];
df = @(t) [cos(t); 2*t; -exp(-t)];

% Create original grid (coarse)
tOld = linspace(0, 2*pi, 10);
yOld = f(tOld);  % Will be a 3xN matrix
dyOld = df(tOld);  % Will be a 3xN matrix

% Create fine grid for testing interpolation
tNew = linspace(0, 2*pi, 100);

% Get true values for comparison
yTrue = f(tNew);
dyTrue = df(tNew);

% Compute interpolation
[yInterp, dyInterp] = spline2(tOld, yOld, dyOld, tNew);

% Compute errors
yError = abs(yInterp - yTrue);
dyError = abs(dyInterp - dyTrue);

% Plot results
figure('Name', 'Interpolation Test');
stateNames = {'sin(t)', 't^2', 'exp(-t)'};

for i = 1:3
    % Plot function values
    subplot(3,2,2*i-1)
    plot(tNew, yTrue(i,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True')
    hold on
    plot(tNew, yInterp(i,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Interpolated')
    plot(tOld, yOld(i,:), 'ko', 'MarkerSize', 8, 'DisplayName', 'Grid Points')
    grid on
    xlabel('t')
    ylabel(['y_' num2str(i)])
    title(['State ' num2str(i) ': ' stateNames{i}])
    legend('Location', 'best')
    
    % Plot derivatives
    subplot(3,2,2*i)
    plot(tNew, dyTrue(i,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True')
    hold on
    plot(tNew, dyInterp(i,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Interpolated')
    plot(tOld, dyOld(i,:), 'ko', 'MarkerSize', 8, 'DisplayName', 'Grid Points')
    grid on
    xlabel('t')
    ylabel(['dy_' num2str(i) '/dt'])
    title(['State ' num2str(i) ' Derivative'])
    legend('Location', 'best')
end

% Print maximum errors
fprintf('Maximum function value error: %.2e\n', max(yError));
fprintf('Maximum derivative error: %.2e\n', max(dyError));

% Verify order of accuracy
h1 = 2*pi/10;  % Original grid spacing
h2 = h1/2;

% Create medium grid
tMed = linspace(0, 2*pi, 19);
yMed = f(tMed);
dyMed = df(tMed);

% Interpolate at test points
tTest = linspace(pi/4, 7*pi/4, 50);
yTrue_test = f(tTest);

[yInterp1, ~] = spline2(tOld, yOld, dyOld, tTest);
[yInterp2, ~] = spline2(tMed, yMed, dyMed, tTest);

err1 = max(abs(yInterp1 - yTrue_test));
err2 = max(abs(yInterp2 - yTrue_test));

order = log(err1/err2)/log(h1/h2);
fprintf('Observed order of accuracy: %.2f\n', order); 