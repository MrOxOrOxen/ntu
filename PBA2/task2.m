m = 1;
l = 0.5;
g = 9.8;
k_list = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5];
Ed = 0;

theta_vec = linspace(-2*pi, 2*pi, 41);
omega_vec = linspace(-10, 10, 41);

[TH, OM] = meshgrid(theta_vec, omega_vec);

E = 0.5*m*l^2 .* OM.^2 + m*g*l .*(cos(TH)-1);
initial = [
    pi-0.1, 0;
    pi+0.1, 0;
    -pi+0.1, 0;
    -pi-0.1, 0;
    % pi, 0.5;
    % -pi, -0.5;
    0.1, 0;
    -0.1, 0;
    % 0, 0.2;
    % 0, -0.2
];

k = 0.2;
k_test = k;
THdot = OM;
OMdot = (g/l).*sin(TH) - (k_test/l).*(E-Ed).*OM.*cos(TH).^2;
mag = sqrt(THdot.^2 + OMdot.^2);
mag(mag == 0) = 1;
U = THdot ./ mag;
V = OMdot ./ mag;

figure(1);
clf;
quiver(TH, OM, U, V, 0.45, 'HandleVisibility', 'off');
hold on;
contour(TH, OM, E, [Ed Ed], ...
    'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Desired energy level E=Ed');
    
dyn = @(t,x) [
    x(2);
    (g/l)*sin(x(1)) - (k_test/l)*(0.5*m*l^2*x(2)^2 + m*g*l*(cos(x(1))-1) - Ed)*x(2)*cos(x(1))^2
    ];
    
    
for i = 1:size(initial, 1)
    x0 = initial(i, :).';
    [t,x] = ode45(dyn, [0 15], x0);
    if i == 1
        plot(x(:,1), x(:,2), 'LineWidth', 1.5, 'DisplayName', 'Closed-loop trajectories');
    else
        plot(x(:,1), x(:,2), 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    % label = sprintf("\\theta_0 = %.2f, d\\theta_0 = %.2f", initial(i, 1), initial(i, 2));
    % plot(x(:,1), x(:,2), 'LineWidth', 1.5, 'DisplayName', label);
end
    
plot(initial(:,1), initial(:,2), 'kx', 'MarkerSize', 7, 'LineWidth', 1, 'DisplayName', 'Initial conditions');
upright_added = false;
downward_added = false;
    
for n = -2:2
    theta_e = n*pi;
    if mod(n,2) == 0
        if ~upright_added
            plot(theta_e, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'DisplayName', 'Upright Equilibria');
            upright_added = true;
        else
            plot(theta_e, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'HandleVisibility', 'off');
        end
    else
        if ~downward_added
            plot(theta_e, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'DisplayName', 'Downward Equilibria');
            downward_added = true;
        else
            plot(theta_e, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'HandleVisibility', 'off');
        end
    end

end

xlabel('$\theta (rad)$', 'Interpreter', 'latex');
ylabel('$\dot{\theta} (rad/s)$', 'Interpreter', 'latex');
title(sprintf("Closed loop phase portrait, k = %.2f, Ed = %.1f J", k_test, Ed));
grid on;

xlim([-2*pi, 2*pi]);
ylim([-10, 10]);
legend('Location', 'best');

hold off;

figure(2);
clf;
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:length(k_list)
    k_test = k_list(j);
    THdot = OM;
    OMdot = (g/l).*sin(TH) - (k_test/l).*(E-Ed).*OM.*cos(TH).^2;
    mag = sqrt(THdot.^2 + OMdot.^2);
    mag(mag == 0) = 1;
    U = THdot ./ mag;
    V = OMdot ./ mag;

    nexttile;
    quiver(TH, OM, U, V, 0.45, 'HandleVisibility', 'off');
    hold on;
    contour(TH, OM, E, [Ed Ed], ...
        'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Desired energy level E=Ed');

    dyn = @(t,x) [
        x(2);
        (g/l)*sin(x(1)) - (k_test/l)*(0.5*m*l^2*x(2)^2 + m*g*l*(cos(x(1))-1) - Ed)*x(2)*cos(x(1))^2
        ];


    for i = 1:size(initial, 1)
        x0 = initial(i, :).';
        [t,x] = ode45(dyn, [0 15], x0);
        if i == 1
            plot(x(:,1), x(:,2), 'LineWidth', 1.5, 'DisplayName', 'Closed-loop trajectories');
        else
            plot(x(:,1), x(:,2), 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
        % label = sprintf("\\theta_0 = %.2f, d\\theta_0 = %.2f", initial(i, 1), initial(i, 2));
        % plot(x(:,1), x(:,2), 'LineWidth', 1.5, 'DisplayName', label);
    end

    plot(initial(:,1), initial(:,2), 'kx', 'MarkerSize', 7, 'LineWidth', 1, 'DisplayName', 'Initial conditions');
    upright_added = false;
    downward_added = false;

    for n = -2:2
        theta_e = n*pi;
        if mod(n,2) == 0
            if ~upright_added
                plot(theta_e, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'DisplayName', 'Upright Equilibria');
                upright_added = true;
            else
                plot(theta_e, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'HandleVisibility', 'off');
            end
        else
            if ~downward_added
                plot(theta_e, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'DisplayName', 'Downward Equilibria');
                downward_added = true;
            else
                plot(theta_e, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'HandleVisibility', 'off');
            end
        end

    end

    xlabel('$\theta (rad)$', 'Interpreter', 'latex');
    ylabel('$\dot{\theta} (rad/s)$', 'Interpreter', 'latex');
    title(sprintf("Closed loop phase portrait, k = %.2f, Ed = %.1f J", k_test, Ed));
    grid on;

    xlim([-2*pi, 2*pi]);
    ylim([-10, 10]);
    hold off;
end

title(tl, 'Effect of controller gain k on the phase portrait');

fprintf('\n');
fprintf('--------------------------------------------\n');
fprintf('Effect of k on downward equilibrium\n');
for j = 1:length(k_list)
    k_test = k_list(j);
    A_down = [0, 1; -g/l, 2*k_test*m*g];
    lambda = eig(A_down);
    fprintf('\nk = %.2f\n', k_test);
    fprintf('Eigenvalues:\n');
    disp(lambda);
end

fprintf('\n');
fprintf('--------------------------------------------\n');
fprintf('Equilibrium Analysis\n');
fprintf('k = %.2f, Ed = %.2f J\n', k, Ed);

for n = -2:2
    theta_e = n*pi;
    omega_e = 0;

    E_e = 0.5*m*l^2*omega_e^2 + m*g*l*(cos(theta_e)-1);
    A = [0, 1; (g/l)*cos(theta_e), -(k/l)*(E_e - Ed)];
    [V_eig, D_eig] = eig(A);
    lambda = diag(D_eig);
    fprintf('Equilibrium: theta = %+.0f*pi rad, omega = 0 rad/s\n', n);
    fprintf('Energy: E_e = %.4f J\n', E_e);
    fprintf('A matrix:\n');
    disp(A);

    fprintf('Eigenvalues:\n');
    disp(lambda);
    tol = 1e-8;

    if all(abs(imag(lambda)) < tol) && real(lambda(1))*real(lambda(2)) < 0
        type = 'Saddle point (unstable)';
    elseif all(real(lambda) > tol)
        if any(abs(imag(lambda)) > tol)
            type = 'Unstable focus';
        else
            type = 'Unstable node';
        end
    elseif all(real(lambda) < -tol)
        if any(abs(imag(lambda)) > tol)
            type = 'Stable focus';
        else
            type = 'Stable node';
        end
    else
        type = 'Further analysis required';
    end
    fprintf('Type: %s\n\n', type);

    % fprintf('Eigenvectors (columns):\n');
    % disp(V_eig);
end