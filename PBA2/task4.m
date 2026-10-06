k = out.k.Data(1);
fprintf('Swing-up performance for k = %.2f\n', k);

energy_threshold = 0.05;
idx = find(abs(out.E.Data) < energy_threshold, 1, 'first');
if ~isempty(idx)
    fprintf("First time to reach |E|<%.2f J: %.2f s\n", energy_threshold, out.E.time(idx));
else
    fprintf("Energy did not reach |E|<%.2f J\n", energy_threshold);
end

[max_F, idx_F] = max(abs(out.F.Data));
time_max_F = out.F.Time(idx_F);
fprintf('Maximum |F|: %.4f N at t = %.4f s\n', max_F, time_max_F);

[max_p, idx_p] = max(abs(out.p.Data));
time_max_p = out.p.Time(idx_p);
fprintf('Maximum |p|: %.4f m at t = %.4f s\n', max_p, time_max_p);

final_E = out.E.Data(end);
fprintf('Final energy: %.6f J\n', final_E);

figure;

tiledlayout(3,1);

nexttile;
plot(out.theta.Time, out.theta.Data, 'LineWidth', 1.5);
yline(2*pi, '--', '2\pi');
yline(0, '--', '0');
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Pendulum Angle');
grid on;

nexttile;
plot(out.E.Time, out.E.Data, 'LineWidth', 1.5);
yline(0, '--');
xlabel('Time (s)');
ylabel('E (J)');
title('Pendulum Energy');
grid on;

nexttile;
plot(out.F.Time, out.F.Data, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('F (N)');
title('Applied Force');
grid on;

sgtitle(sprintf('Swing-up Response for k=%.2f', k));