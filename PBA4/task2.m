clear;
close all;
clc;
bike;
A = [zeros(2), eye(2); -(M\(K0 + v0^2*K2)), -(M\(v0*C0))];
B = [zeros(2,1); M\[0;1]];
C = [0 1 0 0];
D = 0;

% check system stability
controllabilityRank = rank(ctrb(A,B))
pc = [-2 -10 -1+1i -1-1i];
% calculate state feedback gain
K = place(A,B,pc)

% verify eigenvalues
closedLoopPoles = eig(A-B*K)
