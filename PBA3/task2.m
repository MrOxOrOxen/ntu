M = 5;
m = 1;
l = 0.5;
c = 0.2;
gamma = 0;
g = 9.8;

A_analytical = [
    0, 0, 1, 0;
    0, 0, 0, 1;
    0, m*g/M, -c/M, 0;
    0, (M+m)*g/(M*l), -c/(M*l), 0];

B_analytical = [0; 0; 1/M; 1/(M*l)];

fprintf("Analytical linearization from task 1\n");
fprintf("A_analytical = \n");
disp(A_analytical);
fprintf("B_analytical = \n");
disp(B_analytical);

eig_analytical = eig(A_analytical);
fprintf("Eigenvalues of A_analytical = \n");
disp(eig_analytical);

eig_analytical_sorted = sort(eig_analytical);
fprintf("Sorted analytical eigenvalues = \n");
disp(eig_analytical_sorted);

if any(real(eig_analytical) > 0)
    fprintf('The linearized open-loop system is unstable.\n');
elseif all(real(eig_analytical) < 0)
    fprintf('The linearized open-loop system is asymptotically stable.\n');
else
    fprintf('The linearized open-loop system contains eigenvalues ');
    fprintf('on the imaginary axis.\n');
end

model = 'task2_simulink';
load_system(model);

x_eq = zeros(4, 1);
F_eq = 0;
[A_linmod, B_linmod, C_linmod, D_linmod] = linmod(model, x_eq, F_eq);
eig_linmod = eig(A_linmod);
disp('A_linmod = ');
disp(A_linmod);
disp('B_linmod = ');
disp(B_linmod);
disp('Eigenvalues from linmod = ');
disp(eig_linmod);

eig_analytical_sorted = sort(eig(A_analytical));
eig_linmod_sorted = sort(eig(A_linmod));

comparison = table( ...
    eig_analytical_sorted, ...
    eig_linmod_sorted, ...
    abs(eig_analytical_sorted - eig_linmod_sorted), ...
    'VariableNames', ...
    {'Analytical', 'Linmod', 'AbsoluteError'});

disp(comparison);
