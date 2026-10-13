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

Co = ctrb(A, B);
rank_Co = rank(Co);

desired_poles = [-1, -1.5, -2, -2.5];
K = place(A, B, desired_poles);
disp('State-feedback gain K = ');
disp(K);

A_cl = A - B*K;
eig_cl = eig(A_cl);
disp('Closed-loop eigenvalues = ');
disp(eig_cl);

if all(real(eig_cl) < 0)
    disp('The closed-loop system is asymptotically stable.');
else
    disp('The closed-loop system is not asymptotically stable.');
end