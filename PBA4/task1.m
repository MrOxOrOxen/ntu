clear;
close all;
clc;
bike;
% state-space model
A = [zeros(2), eye(2); -(M\(K0 + v0^2*K2)), -(M\(v0*C0))];
B = [zeros(2,1); M\[0;1]];
C = [0 1 0 0];
D = 0;

A
B
C
% check open-loop stability
openPoles = eig(A)
