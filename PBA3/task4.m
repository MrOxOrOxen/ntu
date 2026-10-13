M = 5;
m = 1;
l = 0.5;
c = 0.2;
gamma = 0;
g = 9.8;

A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, m*g/M, -c/M, 0;
     0, (M+m)*g/(M*l), -c/(M*l), 0];

B = [0;
     0;
     1/M;
     1/(M*l)];

desired_poles = [-1 -1.5 -2 -2.5];
K = place(A, B, desired_poles);

% x0 = [p; theta; p_dot; theta_dot]
angles_deg = 1:1:90;
initial_conditions = [
    zeros(length(angles_deg),1), ...
    deg2rad(angles_deg'), ...
    zeros(length(angles_deg),2)
];

initial_conditions = [
    initial_conditions;
    0, deg2rad(5), 0, 0.5
];


model = 'task4_simulink';
load_system(model);

T = 20;
N = size(initial_conditions,1);

results = cell(N,1);
final_states = NaN(N,4);
converged = false(N,1);

for i = 1:N

    x0 = initial_conditions(i,:)';

    simIn = Simulink.SimulationInput(model);
    simIn = simIn.setVariable('x0', x0);
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setModelParameter( ...
        'StopTime', num2str(T));

    try
        simOut = sim(simIn);

        x_ts = simOut.get('x_log');

        results{i} = x_ts;

        x_final = reshape(x_ts.Data(end,:),1,[]);
        final_states(i,:) = x_final;

        t = x_ts.Time;
        X = squeeze(x_ts.Data);
        
        idx_steady = t >= T - 2;
        
        within_tolerance = ...
            abs(X(idx_steady,1)) < 0.02 & ...
            abs(X(idx_steady,2)) < deg2rad(0.5) & ...
            abs(X(idx_steady,3)) < 0.02 & ...
            abs(X(idx_steady,4)) < 0.02;
        
        converged(i) = all(within_tolerance) && all(isfinite(x_final));

        fprintf('Case %d: completed\n', i);

    catch ME
        fprintf('\n========== Case %d failed ==========\n', i);
        fprintf('%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
        rethrow(ME);
    end
end

result_table = table( ...
    (1:N)', ...
    rad2deg(initial_conditions(:,2)), ...
    initial_conditions(:,4), ...
    final_states(:,1), ...
    rad2deg(final_states(:,2)), ...
    final_states(:,3), ...
    final_states(:,4), ...
    converged, ...
    'VariableNames', ...
    {'Case','InitialAngle_deg','InitialOmega', ...
     'FinalP','FinalAngle_deg','FinalPdot', ...
     'FinalOmega','Converged'});

disp(result_table);

n_angles = length(angles_deg);

angle_converged = converged(1:n_angles);

% Largest tested convergent angle
last_converged_idx = find(angle_converged, 1, 'last');

% Smallest tested non-convergent angle
first_failed_idx = find(~angle_converged, 1, 'first');

fprintf('\n===== Convergence Analysis =====\n');

if ~isempty(last_converged_idx)
    fprintf('Largest convergent angle: %.1f deg\n', ...
        angles_deg(last_converged_idx));
end

if ~isempty(first_failed_idx)
    fprintf('First non-convergent angle: %.1f deg\n', ...
        angles_deg(first_failed_idx));
end

% Find all changes in convergence status
transition_idx = find(diff(double(angle_converged)) ~= 0);

fprintf('\nConvergence status transitions:\n');

for j = 1:length(transition_idx)
    k = transition_idx(j);

    fprintf('%.1f deg (%d) -> %.1f deg (%d)\n', ...
        angles_deg(k), angle_converged(k), ...
        angles_deg(k+1), angle_converged(k+1));
end

selected_cases = [10, 50, 60];

for j = 1:length(selected_cases)

    i = selected_cases(j);

    ts = results{i};
    t = ts.Time;
    X = squeeze(ts.Data);

    figure('Name', sprintf('Case %d', i));

    % Cart position
    subplot(2,2,1);
    plot(t, X(:,1), 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('p (m)');
    title('Cart Position');
    grid on;

    subplot(2,2,2);
    plot(t, rad2deg(X(:,2)), 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\theta (deg)');
    title('Pendulum Angle');
    grid on;

    subplot(2,2,3);
    plot(t, X(:,3), 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('dp/dt (m/s)');
    title('Cart Velocity');
    grid on;

    subplot(2,2,4);
    plot(t, X(:,4), 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('d\theta/dt (rad/s)');
    title('Pendulum Angular Velocity');
    grid on;

    sgtitle(sprintf( ...
        'Task 4 - Case %d: Initial Angle = %.0f deg', ...
        i, rad2deg(initial_conditions(i,2))));

end

theta0_deg = rad2deg(initial_conditions(:,2));

% Only plot cases with zero initial angular velocity
idx = initial_conditions(:,4) == 0;

figure;

scatter(theta0_deg(idx), double(converged(idx)), ...
    30, 'filled');

xlabel('Initial Pendulum Angle (deg)');
ylabel('Convergence Status');

yticks([0 1]);
yticklabels({'Not Converged', 'Converged'});

ylim([-0.2 1.2]);
xlim([0 90]);

grid on;
title('Task 4: Convergence for Different Initial Angles');