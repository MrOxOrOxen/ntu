clear;
close all;
clc;
bike;
A = [zeros(2), eye(2); -(M\(K0 + v0^2*K2)), -(M\(v0*C0))];
B = [zeros(2,1); M\[0;1]];
C = [0 1 0 0];
D = 0;

pc = [-2 -10 -1+1i -1-1i];
K = place(A,B,pc);
% calculate reference gain
kr = -1/(C*((A-B*K)\B));

t = (0:0.001:15)';
r = 0.002*ones(size(t));
x0 = zeros(4,1);

% simulate
sys = ss(A-B*K,B*kr,eye(4),zeros(4,1));
x = lsim(sys,r,t,x0);
u = -x*K' + kr*r;

figure;
tiledlayout(3,1);
nexttile;
plot(t,x(:,2),t,r,'--','LineWidth',1.2);
ylabel('\delta (rad)');
legend('Steering angle','Reference');
title('Full-state feedback');
grid on;
nexttile;
plot(t,x(:,1),'LineWidth',1.2);
ylabel('\phi (rad)');
grid on;
nexttile;
plot(t,u,'LineWidth',1.2);
ylabel('T (N m)');
xlabel('Time (s)');
grid on;

mkdir('results');
exportgraphics(gcf,'results/task3_state_feedback.png','Resolution',200);
