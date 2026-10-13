M = 5;
m = 1;
l = 0.5;
c = 0.2;
gamma = 0;
g = 9.8;

A = [0 0 1 0;
     0 0 0 1;
     0 m*g/M -c/M 0;
     0 (M+m)*g/(M*l) -c/(M*l) 0];

B = [0;
     0;
     1/M;
     1/(M*l)];

desired_poles = [-1 -1.5 -2 -2.5];

K = place(A, B, desired_poles);

x0 = [0;
      pi - 0.01;
      0;
      0];

model = 'task5_simulink';
T = 60;

k_values = [2.8];
k_selected = 2.8;
load_system(model);


N = length(k_values);
results = cell(N,1);

min_angle = NaN(N,1);
final_energy = NaN(N,1);
max_abs_p = NaN(N,1);
max_abs_F = NaN(N,1);

final_p = NaN(N,1);
final_theta_e = NaN(N,1);
final_pdot = NaN(N,1);
final_thdot = NaN(N,1);

final_mode = NaN(N,1);
balance_reached = false(N,1);
catch_successful = false(N,1);

first_balance_time = NaN(N,1);

for i = 1:N
    k = k_values(i);

    fprintf('\nRunning k = %.2f ...\n', k);

    simIn = Simulink.SimulationInput(model);

    simIn = simIn.setVariable('x0', x0);
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('k', k);

    simIn = simIn.setModelParameter('StopTime', num2str(T));

    simOut = sim(simIn);
    x_ts = simOut.get('x_log');

    t = x_ts.Time(:);
    X = squeeze(x_ts.Data);

    if size(X,1) == 4 && size(X,2) == length(t)
        X = X';
    end

    if size(X,1) ~= length(t) || size(X,2) ~= 4
        error('Invalid x_log dimensions.');
    end

    p = X(:,1);
    theta = X(:,2);
    pdot = X(:,3);
    thdot = X(:,4);

    theta_e = atan2(sin(theta), cos(theta));

    E = 0.5*m*l^2*thdot.^2 + m*g*l*(cos(theta)-1);

    F_ts = simOut.get('F_log');
    t_F = F_ts.Time(:);
    F = squeeze(F_ts.Data);
    F = F(:);

    mode_ts = simOut.get('mode_log');

    t_mode = mode_ts.Time(:);
    mode_data = squeeze(mode_ts.Data);
    mode_data = mode_data(:);

    results{i} = struct( ...
        'k', k, ...
        't', t, ...
        'X', X, ...
        'theta_e', theta_e, ...
        'E', E, ...
        't_F', t_F, ...
        'F', F, ...
        't_mode', t_mode, ...
        'mode', mode_data);

    idx_low_vel = abs(thdot) < 0.5;

    if any(idx_low_vel)
        min_angle(i) = min(abs(rad2deg(theta_e(idx_low_vel))));
    end

    final_p(i) = p(end);
    final_theta_e(i) = rad2deg(theta_e(end));
    final_pdot(i) = pdot(end);
    final_thdot(i) = thdot(end);

    final_energy(i) = E(end);

    max_abs_p(i) = max(abs(p));
    max_abs_F(i) = max(abs(F));

    final_mode(i) = mode_data(end);

    balance_reached(i) = any(mode_data > 0.5);

    if balance_reached(i)
        idx_balance = find(mode_data > 0.5, 1, 'first');
        first_balance_time(i) = t_mode(idx_balance);
    end

    idx_steady = t >= T - 2;
    state_stable = false;
    if any(idx_steady) && t(end) >= T - 1e-3

        state_stable = ...
            all(abs(p(idx_steady)) < 0.02) && ...
            all(abs(theta_e(idx_steady)) < deg2rad(0.5)) && ...
            all(abs(pdot(idx_steady)) < 0.02) && ...
            all(abs(thdot(idx_steady)) < 0.02);

    end

    idx_mode_steady = t_mode >= T - 2;
    mode_stable = false;

    if any(idx_mode_steady)
        mode_stable = all(mode_data(idx_mode_steady) > 0.5);
    end
    force_valid = max_abs_F(i) <= 14 + 1e-6;

    catch_successful(i) = state_stable && mode_stable && force_valid;

    fprintf('k = %.2f completed.\n', k);

end

gain_comparison = table( ...
    k_values', ...
    min_angle, ...
    final_energy, ...
    max_abs_p, ...
    max_abs_F, ...
    balance_reached, ...
    first_balance_time, ...
    catch_successful, ...
    'VariableNames', ...
    {'k', ...
     'MinAngle_deg', ...
     'FinalEnergy_J', ...
     'MaxAbsP_m', ...
     'MaxAbsF_N', ...
     'BalanceReached', ...
     'FirstBalanceTime_s', ...
     'CatchSuccessful'});

fprintf('\n===== Swing-Up Gain Comparison =====\n');
disp(gain_comparison);

selected_idx = find(abs(k_values - k_selected) < 1e-10, 1);

if isempty(selected_idx)
    error('Selected gain is not in k_values.');
end

R = results{selected_idx};

t = R.t;
X = R.X;

p = X(:,1);
theta = X(:,2);
pdot = X(:,3);
thdot = X(:,4);

theta_e = R.theta_e;
E = R.E;

t_F = R.t_F;
F = R.F;

t_mode = R.t_mode;
mode_data = R.mode;


fprintf('\n===== Task 5 Results (k = %.2f) =====\n', ...
    k_selected);
fprintf('Final cart position: %.6f m\n', p(end));
fprintf('Final wrapped angle: %.6f deg\n', rad2deg(theta_e(end)));
fprintf('Final cart velocity: %.6f m/s\n', pdot(end));
fprintf('Final angular velocity: %.6f rad/s\n', thdot(end));
fprintf('Final energy: %.6f J\n', E(end));
fprintf('Maximum absolute force: %.6f N\n', max(abs(F)));
fprintf('Final controller mode: %d\n', round(mode_data(end)));
fprintf('Balance reached: %d\n', balance_reached(selected_idx));
fprintf('Catch successful: %d\n', catch_successful(selected_idx));

angle_ok = abs(theta_e) < deg2rad(5);
velocity_ok = abs(thdot) < 0.5;

catch_condition = angle_ok & velocity_ok;

fprintf('\n===== Catch Condition Analysis =====\n');
fprintf('Samples satisfying angle condition: %d\n', nnz(angle_ok));
fprintf('Samples satisfying velocity condition: %d\n', nnz(velocity_ok));
fprintf('Samples satisfying both conditions: %d\n', nnz(catch_condition));

if any(velocity_ok)
    min_angle_low_vel = min(abs(rad2deg(theta_e(velocity_ok))));
    fprintf('Minimum angle error at low velocity: %.4f deg\n', min_angle_low_vel);
end


figure('Name', 'Task 5 - State Response');

subplot(2,2,1);
plot(t, p, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('p (m)');
title('Cart Position');
grid on;

subplot(2,2,2);
plot(t, rad2deg(theta), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Pendulum Angle');
grid on;

subplot(2,2,3);
plot(t, pdot, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('dp/dt (m/s)');
title('Cart Velocity');
grid on;

subplot(2,2,4);
plot(t, thdot, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\theta/dt (rad/s)');
title('Pendulum Angular Velocity');
grid on;

sgtitle(sprintf( ...
    'Task 5: State Response (k = %.2f)', k_selected));


figure('Name', 'Task 5 - Controller Analysis');

subplot(3,1,1);
plot(t, E, 'LineWidth', 1.5);
yline(0, '--');
xlabel('Time (s)');
ylabel('Energy (J)');
title('Pendulum Energy');
grid on;

subplot(3,1,2);
plot(t_F, F, 'LineWidth', 1.5);
yline(14, '--');
yline(-14, '--');
xlabel('Time (s)');
ylabel('Force (N)');
title('Applied Control Force');
grid on;

subplot(3,1,3);
stairs(t_mode, mode_data, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Mode');
yticks([0 1]);
yticklabels({'Swing-Up', 'Balance'});
ylim([-0.2 1.2]);
title('Stateflow Controller Mode');
grid on;

sgtitle(sprintf('Task 5: Controller Analysis (k = %.2f)', k_selected));
