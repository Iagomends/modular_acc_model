% Load data
data = load('AMT_06_M142_P60.mat');
disp(fieldnames(data)); % Show available variables

% Select data
y = data.data.accel_pedal;
t = 1:length(y);
t = t/1000;

% Plot
figure;
plot(t, y, 'b-', 'LineWidth', 1.5,'Color', [0.8500 0.3250 0.0980]);

xlabel('Time [s]');
ylabel('Pedal [%]');
legend('Pedal', 'Location', 'northwest');

xlim([1 7]);          % X-axis limits
ylim([0 80]);           % Y-axis limits

grid on;
ax = gca;