fprintf("### TASK 2 ###\n")
fprintf("Gear: 1\n");
fprintf("Throttle u: 0.5\n");
fprintf("Road slope: %f deg\n", theta);

t = out.v_out.time;
v = out.v_out.signals.values;

% v_stable = v(end);
v_stable = mean(v(round(0.95 * end) : end));
fprintf("Steady-state speed: %f m/s\n", v_stable);

v_5 = 0.05 * v_stable;
v_95 = 0.95 * v_stable;
fprintf("5%% of speed: %f m/s\n", v_5);
fprintf("95%% of speed: %f m/s\n", v_95);

idx_5 = find(v >= v_5, 1, 'first');
idx_95 = find(v >= v_95, 1, 'first');

t_5 = t(idx_5);
t_95 = t(idx_95);

fprintf("Time for 5%% speed: %.2f s\n", t_5);
fprintf("Time for 95%% speed: %.2f s\n", t_95);
fprintf("Rise time: %.2f s\n", t_95 - t_5)

figure;
hold on;

plot(t, v, 'r-', 'LineWidth', 1.8, 'DisplayName', 'v(t)');

xlabel('Time (s)', 'FontSize', 13, 'Color', 'k');
ylabel('Speed (m/s)', 'FontSize', 13);
title('Task 2: Velocity of Open-loop Control', 'FontSize', 14, 'Color', 'k');
set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);

grid on;
box on;
xlim([0, 70]);
ylim([0, 40]);

print(gcf, 'task2/task2.png', '-dpng', '-r300');

headers_raw = {'Time (s)', 'v (m/s)'};
data_raw = num2cell([t, v]);
if ~isfile('task2/task2.xlsx')
    writecell([headers_raw; data_raw], 'task2/task2.xlsx', 'Sheet', 'Raw');
else
    writecell(data_raw, 'task2/task2.xlsx', 'Sheet', 'Raw', 'WriteMode', 'append');
end