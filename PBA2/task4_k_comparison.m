outs = {out1, out2, out4};
k_list = [0.1, 0.2, 0.4];

energy_threshold = 0.05;

T_E = zeros(length(k_list), 1);
max_F_all = zeros(length(k_list), 1);
max_p_all = zeros(length(k_list), 1);
final_E_all = zeros(length(k_list), 1);

for j = 1:length(k_list)
    out_current = outs{j};
    k = k_list(j);
    fprintf('Swing-up performance for k = %.2f\n', k);

    idx = find(abs(out_current.E.Data) < energy_threshold, 1, 'first');

    if ~isempty(idx)
        T_E(j) = out_current.E.Time(idx);
        fprintf('First time to reach |E| < %.2f J: %.2f s\n', energy_threshold, T_E(j));
    else
        T_E(j) = NaN;
        fprintf('Energy did not reach |E| < %.2f J\n', energy_threshold);
    end

    [max_F_all(j), idx_F] = max(abs(out_current.F.Data));
    time_max_F = out_current.F.Time(idx_F);
    fprintf( 'Maximum |F|: %.4f N at t = %.4f s\n', max_F_all(j), time_max_F);

    [max_p_all(j), idx_p] = max(abs(out_current.p.Data));
    time_max_p = out_current.p.Time(idx_p);
    fprintf('Maximum |p|: %.4f m at t = %.4f s\n', max_p_all(j), time_max_p);
    final_E_all(j) = out_current.E.Data(end);
    fprintf( 'Final energy: %.6f J\n', final_E_all(j));
end


comparison_table = table( ...
    k_list.', ...
    T_E, ...
    max_F_all, ...
    max_p_all, ...
    final_E_all, ...
    'VariableNames', { ...
        'k', ...
        'FirstTime_E_Below_005_s', ...
        'MaxAbsForce_N', ...
        'MaxAbsPosition_m', ...
        'FinalEnergy_J' ...
    });

fprintf('Summary of k comparison\n');
disp(comparison_table);

figure;

tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;

for j = 1:length(k_list)
    out_current = outs{j};
    plot( ...
        out_current.theta.Time, ...
        out_current.theta.Data, ...
        'LineWidth', 1.5, ...
        'DisplayName', ...
        sprintf('k = %.1f', k_list(j)));
end

yline(0, '--', '0', 'HandleVisibility', 'off');
yline(2*pi, '--', '2\pi', 'HandleVisibility', 'off');

xlabel('Time (s)');
ylabel('\theta (rad)');
title('Pendulum Angle');

grid on;
legend('Location', 'best');

hold off;

nexttile;
hold on;

for j = 1:length(k_list)
    out_current = outs{j};
    plot( ...
        out_current.E.Time, ...
        out_current.E.Data, ...
        'LineWidth', 1.5);

end

yline(0, '--', 'HandleVisibility', 'off');

xlabel('Time (s)');
ylabel('E (J)');
title('Pendulum Energy');

grid on;

hold off;

nexttile;
hold on;

for j = 1:length(k_list)
    out_current = outs{j};
    plot( ...
        out_current.F.Time, ...
        out_current.F.Data, ...
        'LineWidth', 1.5);

end

xlabel('Time (s)');
ylabel('F (N)');
title('Applied Force');

grid on;

hold off;

nexttile;
hold on;

for j = 1:length(k_list)
    out_current = outs{j};
    plot( ...
        out_current.p.Time, ...
        out_current.p.Data, ...
        'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('p (m)');
title('Cart Position');

grid on;

hold off;

title(tl, 'Effect of Controller Gain k on Swing-up Performance');