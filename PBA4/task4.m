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
kr = -1/(C*((A-B*K)\B));

% check observability and design state observer
observabilityRank = rank(obsv(A,C))
po = [-8 -16 -4+1i -4-1i];
L = place(A',C',po)'
observerPoles = eig(A-L*C)

t = (0:0.001:15)';
r = 0.002*ones(size(t));
x0 = zeros(4,1);
xhat0 = [0.01;0;0;0];

% construct the combined plant-observer system
Aa = [A, -B*K; L*C, A-B*K-L*C];
Ba = [B*kr; B*kr];
sys = ss(Aa,Ba,eye(8),zeros(8,1));
z = lsim(sys,r,t,[x0;xhat0]);
x = z(:,1:4);
xhat = z(:,5:8);
% calculate errors
e = x-xhat;
u = -xhat*K' + kr*r;

sysSF = ss(A-B*K,B*kr,eye(4),zeros(4,1));
xSF = lsim(sysSF,r,t,x0);
uSF = -xSF*K' + kr*r;

labels = {'\phi (rad)','\delta (rad)','d\phi/dt (rad/s)','d\delta/dt (rad/s)'};
mkdir('results');
figure;
tiledlayout(2,2);
for k = 1:4
    nexttile;
    plot(t,x(:,k),t,xhat(:,k),'--','LineWidth',1.2);
    ylabel(labels{k});
    xlabel('Time (s)');
    legend('Actual','Estimated');
    grid on;
end
exportgraphics(gcf,'results/task4_actual_and_estimated_states.png','Resolution',200);

figure;
tiledlayout(2,2);
for k = 1:4
    nexttile;
    plot(t,e(:,k),'LineWidth',1.2);
    ylabel(['Error: ' labels{k}]);
    xlabel('Time (s)');
    xlim([0 2]);
    grid on;
end
exportgraphics(gcf,'results/task4_estimation_errors.png','Resolution',200);

figure;
tiledlayout(2,1);
nexttile;
plot(t,xSF(:,2),t,x(:,2),'--',t,r,':','LineWidth',1.2);
ylabel('\delta (rad)');
legend('Full-state feedback','Observer feedback','Reference');
grid on;
nexttile;
plot(t,uSF,t,u,'--','LineWidth',1.2);
ylabel('T (N m)');
xlabel('Time (s)');
legend('Full-state feedback','Observer feedback');
grid on;
exportgraphics(gcf,'results/task4_feedback_comparison.png','Resolution',200);
