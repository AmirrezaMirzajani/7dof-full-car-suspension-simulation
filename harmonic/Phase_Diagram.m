function out = Phase_Diagram(X, X_dot,X_2dot)
   fig = figure;

% Subplot 1: Phase diagram for Position vs Velocity
subplot(1, 2, 1); % 1 row, 2 columns, first plot
plot(X, X_dot, 'r-', 'LineWidth', 1.5); % Position vs Velocity with red line
hold on;
plot(0, 0, 'bo', 'MarkerFaceColor', 'b'); % Equilibrium point at (0, 0)
title('Phase Diagram: Position vs Velocity', 'Interpreter', 'latex');
xlabel('Position (m)', 'Interpreter', 'latex');
ylabel('Velocity (m/s)', 'Interpreter', 'latex');
xlim([min(X) max(X)]);
ylim([min(X_dot) max(X_dot)]);
grid on;
text(0, 0, 'Equilibrium', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
     'FontSize', 10, 'FontWeight', 'bold');

% Subplot 2: Phase diagram for Velocity vs Acceleration
subplot(1, 2, 2); % 1 row, 2 columns, second plot
plot(X_dot, X_2dot, 'b-', 'LineWidth', 1.5); % Velocity vs Acceleration with blue line
hold on;
plot(0, 0, 'ro', 'MarkerFaceColor', 'r'); % Equilibrium point at (0, 0)
title('Phase Diagram: Velocity vs Acceleration', 'Interpreter', 'latex');
xlabel('Velocity (m/s)', 'Interpreter', 'latex');
ylabel('Acceleration (m/s^2)', 'Interpreter', 'latex');
xlim([min(X_dot) max(X_dot)]);
ylim([min(X_2dot) max(X_2dot)]);
grid on;
text(0, 0, 'Equilibrium', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
     'FontSize', 10, 'FontWeight', 'bold');

% Output figure with both plots

    
    % Output the figure handle
    out = fig;
end
