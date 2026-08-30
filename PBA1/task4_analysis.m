fprintf("### TASK 4 ###\n");
fprintf("Road Slope: %f deg\n", theta);
fprintf("Throttle u: 1\n");

t = out.v_out.time;
v = out.v_out.signals.values;
w = out.w_out.signals.values;
gear = out.gear_out.signals.values;

v_100 = 100 / 3.6;
idx_100 = find(v >= v_100, 1, 'first');
t_100 = t(idx_100);
fprintf("Time of 100 km/h: %f s\n", t_100);

idx_g2 = find(gear == 2, 1, 'first');
idx_g3 = find(gear == 3, 1, 'first');
idx_g4 = find(gear == 4, 1, 'first');
idx_g5 = find(gear == 5, 1, 'first');
t_g2 = t(idx_g2);
t_g3 = t(idx_g3);
t_g4 = t(idx_g4);
t_g5 = t(idx_g5);
fprintf("Time of gear 1->2: %f s\n", t_g2);
fprintf("Time of gear 2->3: %f s\n", t_g3);
fprintf("Time of gear 3->4: %f s\n", t_g4);
fprintf("Time of gear 4->5: %f s\n", t_g5);

v_kmh = v * 3.6;
% fig 1
figure;
subplot(3, 1, 1);

plot(t, v_kmh, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Vehicle velocity v(t)');
hold on;

yline(100, 'k--', 'LineWidth', 1.5, 'DisplayName', '100 km/h');
xline(t_g2, 'k--', '1 \rightarrow 2', 'LineWidth', 1.5);
xline(t_g3, 'k--', '2 \rightarrow 3', 'LineWidth', 1.5);
xline(t_g4, 'k--', '3 \rightarrow 4', 'LineWidth', 1.5);
xline(t_g5, 'k--', '4 \rightarrow 5', 'LineWidth', 1.5);

ylabel("Velocity (km/h)", 'FontSize', 13);
xlabel("Time (s)", 'FontSize', 13);
title("Vehicle Velocity", 'FontSize', 14, 'Color', 'k');

grid on;
box on;
xlim([0, t(end) + 1]);

set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);

subplot(3, 1, 2);
plot(t, w, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Engine Rotational Speed');
hold on;

xline(t_g2, 'k--', '1 \rightarrow 2', 'LineWidth', 1.5);
xline(t_g3, 'k--', '2 \rightarrow 3', 'LineWidth', 1.5);
xline(t_g4, 'k--', '3 \rightarrow 4', 'LineWidth', 1.5);
xline(t_g5, 'k--', '4 \rightarrow 5', 'LineWidth', 1.5);

ylabel("Engine Speed (rad/s)", 'FontSize', 13);
xlabel("Time (s)", 'FontSize', 13);
title("Engine Rotational Speed", 'FontSize', 14, 'Color', 'k');

grid on;
box on;
xlim([0, t(end) + 1]);

set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);

subplot(3, 1, 3);
stairs(t, gear, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Gear');
hold on;

xline(t_g2, 'k--', '1 \rightarrow 2', 'LineWidth', 1.5);
xline(t_g3, 'k--', '2 \rightarrow 3', 'LineWidth', 1.5);
xline(t_g4, 'k--', '3 \rightarrow 4', 'LineWidth', 1.5);
xline(t_g5, 'k--', '4 \rightarrow 5', 'LineWidth', 1.5);

ylabel("Gear", 'FontSize', 13);
xlabel("Time (s)", 'FontSize', 13);
title("Active Gear", 'FontSize', 14, 'Color', 'k');

yticks(1:5);
ylim([0, 5.5]);

grid on;
box on;
xlim([0, t(end) + 1]);

set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);
sgtitle("Task 4: Vehicle Speed, Engine Rotational Speed and Active Gear", 'Color', 'k', 'FontSize', 15);

print(gcf, 'task4/task4_100s.png', '-dpng', '-r300');

% fig 2
figure;
subplot(3, 1, 1);

plot(t, v_kmh, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Vehicle velocity v(t)');
hold on;

yline(100, 'k--', 'LineWidth', 1.5, 'DisplayName', '100 km/h');
xline(t_g2, 'k--', '1 \rightarrow 2', 'LineWidth', 1.5);
xline(t_g3, 'k--', '2 \rightarrow 3', 'LineWidth', 1.5);
xline(t_g4, 'k--', '3 \rightarrow 4', 'LineWidth', 1.5);
xline(t_g5, 'k--', '4 \rightarrow 5', 'LineWidth', 1.5);

ylabel("Velocity (km/h)", 'FontSize', 13);
xlabel("Time (s)", 'FontSize', 13);
title("Vehicle Velocity", 'FontSize', 14, 'Color', 'k');

grid on;
box on;
xlim([0, 20]);

set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);

subplot(3, 1, 2);
plot(t, w, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Engine Rotational Speed');
hold on;

xline(t_g2, 'k--', '1 \rightarrow 2', 'LineWidth', 1.5);
xline(t_g3, 'k--', '2 \rightarrow 3', 'LineWidth', 1.5);
xline(t_g4, 'k--', '3 \rightarrow 4', 'LineWidth', 1.5);
xline(t_g5, 'k--', '4 \rightarrow 5', 'LineWidth', 1.5);

ylabel("Engine Speed (rad/s)", 'FontSize', 13);
xlabel("Time (s)", 'FontSize', 13);
title("Engine Rotational Speed", 'FontSize', 14, 'Color', 'k');

grid on;
box on;
xlim([0, 20]);

set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);

subplot(3, 1, 3);
stairs(t, gear, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Gear');
hold on;

xline(t_g2, 'k--', '1 \rightarrow 2', 'LineWidth', 1.5);
xline(t_g3, 'k--', '2 \rightarrow 3', 'LineWidth', 1.5);
xline(t_g4, 'k--', '3 \rightarrow 4', 'LineWidth', 1.5);
xline(t_g5, 'k--', '4 \rightarrow 5', 'LineWidth', 1.5);

ylabel("Gear", 'FontSize', 13);
xlabel("Time (s)", 'FontSize', 13);
title("Active Gear", 'FontSize', 14, 'Color', 'k');

yticks(1:5);
ylim([0, 5.5]);

grid on;
box on;
xlim([0, 20]);

set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);
sgtitle("Task 4: Vehicle Speed, Engine Rotational Speed and Active Gear (0-20s)", 'Color', 'k', 'FontSize', 15);

print(gcf, 'task4/task4_20s.png', '-dpng', '-r300');

headers_raw = {'Time (s)', 'Gear', 'Engine w (rad/s)', 'Vehicle v (km/h)', 'Vehicle v (m/s)'};
data_raw = num2cell([t, gear, w, v_kmh, v]);
if ~isfile('task4/task4.xlsx')
    writecell([headers_raw; data_raw], 'task4/task4.xlsx', 'Sheet', 'Raw');
else
    writecell(data_raw, 'task4/task4.xlsx', 'Sheet', 'Raw', 'WriteMode', 'append');
end