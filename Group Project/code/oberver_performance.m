A = [0 0 1 0;
     0 0 0 1;
     0 26.6844 -9.4482 -0.3055;
     0 69.4005 -9.0917 -0.7945];

B = [0;
     0;
     16.9843;
     16.3434];

C = [1 0 0 0;
     0 1 0 0];

G = [0 0;
     0 0;
     1 0;
     0 1];

Qn = 10*eye(2);
Rn = eye(2);

L = lqe(A, G, C, Qn, Rn);

A_obs = A - L*C;
B_obs = [B L];
C_obs = eye(4);
D_obs = zeros(4,3);


q_values = logspace(-2, 3, 40);

R = eye(2);
e0 = [0.1; -0.05; 0.3; -0.2];
t = (0:0.05:60)';

settling_time = nan(size(q_values));
peak_error = zeros(size(q_values));
slowest_pole = zeros(size(q_values));

for k = 1:length(q_values)

    Q = q_values(k)*eye(2);

    L = lqe(A, G, C, Q, R);

    Ae = A - L*C;

    % Slowest observer pole
    slowest_pole(k) = max(real(eig(Ae)));


    sys = ss(Ae, zeros(4,1), ...
             eye(4), zeros(4,1));

    E = initial(sys, e0, t);
    err = abs(E(:,4));

    %Maximum absolute error
    peak_error(k) = max(err);

    %2% settling threshold
    threshold = 0.02*abs(e0(4));

    last_idx = find(err > threshold, 1, 'last');

    if isempty(last_idx)
        settling_time(k) = 0;
    elseif last_idx < length(t)
        settling_time(k) = t(last_idx+1);
    end
end

figure;
tiledlayout(3,1);

nexttile;
semilogx(q_values, settling_time, 'LineWidth', 1.5);
grid on;
ylabel('Settling Time (s)');

nexttile;
semilogx(q_values, peak_error, 'LineWidth', 1.5);
grid on;
ylabel('Peak Error (rad/s)');

nexttile;
semilogx(q_values, slowest_pole, 'LineWidth', 1.5);
grid on;
ylabel('Slowest Pole');
xlabel('Q/R Ratio');

sgtitle('LQE Parameter Sensitivity Analysis');
