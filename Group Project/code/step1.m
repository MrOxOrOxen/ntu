function [plant, observer] = step1
% Preserve the supplied upright linearization and four-state LQE design.
plant_parameters;
plant = struct('mp',mp,'r',r,'Lp',Lp,'lp',lp,'J',J,'Jp',Jp, ...
    'eta_g',eta_g,'G',G,'eta_m',eta_m,'kt',kt,'Ra',Ra,'kb',kb, ...
    'Bp',Bp,'Br',Br,'g',g);
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
C = [1 0 0 0; 0 1 0 0];
D = zeros(2,1);
Gnoise = [zeros(2); eye(2)];
Qn = 10*eye(2);
Rn = eye(2);
L = lqe(A,Gnoise,C,Qn,Rn);
A_obs = A-L*C;
B_obs = [B L];
C_obs = eye(4);
D_obs = zeros(4,3);
assert(rank(ctrb(A,B))==4 && rank(obsv(A,C))==4);
assert(all(real(eig(A_obs))<0));
observer = struct('A',A,'B',B,'C',C,'D',D,'Gnoise',Gnoise, ...
    'Qn',Qn,'Rn',Rn,'L',L,'A_obs',A_obs,'B_obs',B_obs,'C_obs',C_obs,'D_obs',D_obs);
fprintf('STEP1_OK | controllability 4/4 | observability 4/4\n');
disp('LQE gain L ='); disp(L);
end
