cd(fileparts(mfilename('fullpath')));
AAP5_setup;

Marm = J + mp*r^2;
Hpend = Jp + mp*lp^2;
couple = mp*lp*r;
hgrav = mp*lp*g;
kVoltage = eta_g*G*eta_m*kt/Ra;
dampingVoltage = Br + kVoltage*G*kb;
Delta = Marm*Hpend - couple^2;
assert(Delta > 0, 'Linearized mass matrix is singular/nonphysical.');

A = [0, 0,                     1,                       0;
     0, 0,                     0,                       1;
     0, couple*hgrav/Delta,   -Hpend*dampingVoltage/Delta, -couple*Bp/Delta;
     0, Marm*hgrav/Delta,     -couple*dampingVoltage/Delta, -Marm*Bp/Delta];
B = [0; 0; Hpend*kVoltage/Delta; couple*kVoltage/Delta];
C = [1, 0, 0, 0;
     0, 1, 0, 0];
D = zeros(2,1);


disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);
fprintf('rank(ctrb(A,B)) = %d of 4\n', rank(ctrb(A,B)));
fprintf('rank(obsv(A,C)) = %d of 4\n', rank(obsv(A,C)));
disp('Open-loop poles ='); disp(eig(A));


Gnoise = [0, 0;
          0, 0;
          1, 0;
          0, 1];
Qn = 10*eye(2);
Rn = eye(2);

L = lqe(A, Gnoise, C, Qn, Rn);
disp('LQE gain L (4-by-2) ='); disp(L);
disp('Observer error poles eig(A-L*C) ='); disp(eig(A-L*C));
assert(all(real(eig(A-L*C)) < 0), 'Observer error dynamics are unstable.');

A_obs = A - L*C;               
B_obs = [B, L];                
C_obs = eye(4);                
D_obs = zeros(4,3);            


disp('A_obs ='); disp(A_obs);
disp('B_obs ='); disp(B_obs);
disp('C_obs ='); disp(C_obs);
disp('D_obs ='); disp(D_obs);


save('MA6217_LQE_matrices.mat', 'A','B','C','D', 'Gnoise','Qn','Rn','L', ...
     'A_obs','B_obs','C_obs','D_obs');
disp('Saved MA6217_LQE_matrices.mat');

