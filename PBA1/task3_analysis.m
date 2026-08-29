fprintf("### TASK 3 ###\n");
fprintf("Gear: 3\n");
fprintf("Road slope: %f deg\n", theta)
fprintf("K_p: %.3f\n", k_p);
fprintf("K_i: %.3f\n", k_i);

t = out.v_out.time;
v = out.v_out.signals.values;

% v_stable = v(end);
v_stable = mean(v(round(0.95 * end) : end));
v_max = max(v);
idx_max = find(v == v_max, 1, 'first');
t_max = t(idx_max);
v_start = 20;
fprintf("Steady-state speed: %f m/s\n", v_stable);
fprintf("Max speed: %f m/s\n", v_max);

overshoot = (v_max - v_stable) / v_stable * 100;
fprintf("Overshoot: %f %%\n", overshoot);

v_5 = v_start + 0.05 * (v_stable - v_start);
v_95 = v_start + 0.95 * (v_stable - v_start);
    
idx_5 = find(v >= v_5, 1, 'first');
idx_95 = find(v >= v_95, 1, 'first');

t_5 = t(idx_5);
t_95 = t(idx_95);
t_r = t_95 - t_5;
fprintf("Rise time: %.2f s\n", t_r)

band = 0.05 * v_stable;
in_band = (v <= v_stable + band) & (v >= v_stable - band);
idx_settling = find(~in_band, 1, 'last');

if isempty(idx_settling)
    t_s_5 = 0;
else
    t_s_5 = t(idx_settling);
end
fprintf("Settling Time (5%%): %.2f s\n", t_s_5)

band = 0.02 * v_stable;
in_band = (v <= v_stable + band) & (v >= v_stable - band);
idx_settling = find(~in_band, 1, 'last');

if isempty(idx_settling)
    t_s_2 = 0;
else
    t_s_2 = t(idx_settling);
end
fprintf("Settling Time (2%%): %.2f s\n", t_s_2)

figure;
hold on;

plot(t, v, 'r-', 'LineWidth', 1.8, 'DisplayName', 'v(t)');
% plot([t(1), t(end)], [v_stable, v_stable], 'r--', 'LineWidth', 0.8);
% plot([t(1), t(end)], [v_5, v_5], 'r:', 'LineWidth', 0.8);
% plot([t(1), t(end)], [v_95, v_95], 'r:', 'LineWidth', 0.8);
% 
% plot([t_5, t_5], [0, v_5], 'r:', 'LineWidth', 0.8);
% plot([t_95, t_95], [0, v_95], 'r:', 'LineWidth', 0.8);
% plot([t_5, t_95], [0, 0], 'r:', 'LineWidth', 0.8);
% 
% plot([t_max, t_max], [0, v_max], 'r:', 'LineWidth', 0.8);

xlabel('Time (s)', 'FontSize', 13);
ylabel('Speed (m/s)', 'FontSize', 13);
title(sprintf('Task 3: Velocity of Closed-loop PI Control (Kp=%.3f, Ki=%.3f)' , k_p, k_i), 'FontSize', 14, 'Color', 'k');
set(gcf, 'Color', 'w');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.5, 0.5, 0.5]);

grid on;
box on;
xlim([0, 70]);
ylim([0, 40]);

print(gcf, 'task3/task3.png', '-dpng', '-r300');

headers = {
    'K_p', 'K_i', 'Steady-state speed (m/s)', 'Max speed (m/s)', 'Overshoot (%)', ...
    'Rise time (s)', '5% Settling Time (s)', '2% Settling Time (s)'
};
data = num2cell([k_p, k_i, v_stable, v_max, overshoot, t_r, t_s_5, t_s_2]);
% writecell([headers; data], 'task3.xlsx', 'Sheet', 'Summary');
if ~isfile('task3/task3.xlsx')
    writecell([headers; data], 'task3/task3.xlsx', 'Sheet', 'Summary');
    writecell([headers_raw; data_raw], 'task3/task3.xlsx', 'Sheet', 'Raw');
else
    writecell(data, 'task3/task3.xlsx', 'Sheet', 'Summary', 'WriteMode', 'append');
end

kp_column = k_p * ones(size(t));
ki_column = k_i * ones(size(t));
headers_raw = {'K_p', 'K_i', 'Time (s)', 'v (m/s)'};
data_raw = num2cell([kp_column, ki_column, t, v]);
writecell(data_raw, 'task3/task3.xlsx', 'Sheet', 'Raw', 'WriteMode', 'append');
