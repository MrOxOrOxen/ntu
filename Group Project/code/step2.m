clc;
MA6217_intial;
load('MA6217_LQE_matrices.mat', 'A', 'B');

t = 0:0.01:5;
x0 = [0; 5*pi/180; 0; 0];

Q_values = [10 100 1 10; 10 300 1 10; 10 100 1 10; 10 100 1 10];
R_values = [1 1 0.1 10];
labels = {'Baseline', 'Higher Q_alpha', 'Lower R', 'Higher R'};
K_all = zeros(4,4);

figure(2);clf;
for i = 1:4
    Q = diag(Q_values(i,:));
    R = R_values(i);
    K = lqr(A, B, Q, R);
    K_all(i,:) = K;
    Acl = A - B*K;
    poles = eig(Acl);
    sys_cl = ss(Acl, zeros(4,1), eye(4), zeros(4,1));
    [x, tout] = initial(sys_cl, x0, t);
    u = -x*K';
    fprintf('\n%s\n', labels{i});
    fprintf('K = %s\n', mat2str(K,4));
    fprintf('Eigenvalues = %s\n', mat2str(poles.',4));
    fprintf('Max arm angle = %.2f deg\n', max(abs(x(:,1)))*180/pi);
    fprintf('Max voltage = %.2f V\n', max(abs(u)));
    subplot(3,1,1); plot(tout, x(:,1)*180/pi, 'DisplayName', labels{i}); hold on;
    subplot(3,1,2); plot(tout, x(:,2)*180/pi, 'DisplayName', labels{i}); hold on;
    subplot(3,1,3); plot(tout, u, 'DisplayName', labels{i}); hold on;
end

subplot(3,1,1); ylabel('\theta (deg)'); grid on; legend('Location','best');
subplot(3,1,2); ylabel('\alpha (deg)'); grid on;
subplot(3,1,3); ylabel('Voltage (V)'); xlabel('Time (s)'); grid on;
sgtitle('LQR Weight Comparison');

selected_case = 2;
K = K_all(selected_case,:);
disp('Selected LQR gain K:');
disp(K);

[armPartPatchHandle, armPartVertices, pendPartPatchHandle, pendPartVertices, drivePartPatchHandle, drivePartVertices, thetaPlotHandle, alphaPlotHandle] = initialiseVisualiser(10);