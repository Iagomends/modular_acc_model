% Load data
data = load('acc_with_shaft.mat');
data1 = load('acc_without_shaft.mat');
disp(fieldnames(data)); % Show available variables

% Select data
y = data.out.logsout.get('acc').Values;
y1 = data1.out.logsout.get('acc').Values;
%t = 1:length(data);
%t = t/1000;

% Plot
figure;
plot(y.Time, y.Data, 'b-', 'LineWidth', 1.5,'Color', [0.8500 0.3250 0.0980]); hold on;
plot(y1.Time, y1.Data, 'b-', 'LineWidth', 1.5,'Color', [0.00 0.36 0.65], 'LineStyle', '--')
legend('Acceleration with dynamic shaft', 'Acceleration without dynamic shaft', 'Location', 'northwest');

xlabel('Time [s]');
ylabel('acceleration [m/s²]');



xlim([0 0.1]);          % X-axis limits
ylim([0 10]);           % Y-axis limits

grid on;
ax = gca;