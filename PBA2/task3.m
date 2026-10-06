figure;
tiledlayout(2,1);

nexttile;
plot(out.p.Time, out.p.Data, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Cart position p (m)');
title('Cart Position');
grid on;

% figure;
nexttile;

plot(out.theta.Time, out.theta.Data, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('\theta (rad)');
title('Pendulum Angle');
grid on;

sgtitle('Model Validation under a Constant Force F=1N');