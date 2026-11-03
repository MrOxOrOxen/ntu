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

t = (0:0.001:15)';
r = 0.002*ones(size(t));
x0 = zeros(4,1);
xhat0 = [0.01;0;0;0];

% define observer poles
po = [-8 -16 -4+1i -4-1i];
alpha = [0.25 0.5 1 2 4];
tol = [1e-4 1e-4 1e-3 1e-3];
delta = zeros(length(t),5);
torque = zeros(length(t),5);
errorNorm = zeros(length(t),5);
metrics = zeros(5,6);

% simulate
for k = 1:5
    L = place(A',C',alpha(k)*po)';
    Aa = [A, -B*K; L*C, A-B*K-L*C];
    Ba = [B*kr; B*kr];
    sys = ss(Aa,Ba,eye(8),zeros(8,1));
    z = lsim(sys,r,t,[x0;xhat0]);
    e = z(:,1:4)-z(:,5:8);
    delta(:,k) = z(:,2);
    torque(:,k) = -z(:,5:8)*K' + kr*r;
    % normalize state errors
    errorNorm(:,k) = max(abs(e)./tol,[],2);
    % calculate settling times, peak errors, torque and gain norm
    te = t(find(errorNorm(:,k)>1,1,'last')+1);
    td = t(find(abs(delta(:,k)-r)>0.02*0.002,1,'last')+1);
    metrics(k,:) = [alpha(k),te,td,max(abs(delta(:,k)-r)),max(abs(torque(:,k))),norm(L)];
end

names = {'Alpha','ErrorSettling_s','SteeringSettling_s','PeakTrackingError_rad','PeakTorque_Nm','ObserverGainNorm'};
resultTable = array2table(metrics,'VariableNames',names)
mkdir('results');
writetable(resultTable,'results/task5_metrics.csv');

legends = {'alpha = 0.25','alpha = 0.5','alpha = 1','alpha = 2','alpha = 4'};
figure;
tiledlayout(3,1);
nexttile;
plot(t,delta,'LineWidth',1.2);
hold on;
plot(t,r,'k--');
ylabel('\delta (rad)');
legend([legends,{'Reference'}]);
grid on;
nexttile;
semilogy(t,errorNorm,'LineWidth',1.2);
yline(1,'k--','Tolerance');
ylim([1e-6 1e3]);
ylabel('Normalized error');
grid on;
nexttile;
plot(t,torque,'LineWidth',1.2);
ylabel('T (N m)');
xlabel('Time (s)');
grid on;
exportgraphics(gcf,'results/task5_observer_pole_comparison.png','Resolution',200);

figure;
tiledlayout(2,1);
nexttile;
plot(t,delta(:,3:5),'LineWidth',1.2);
hold on;
plot(t,r,'k--');
ylabel('\delta (rad)');
legend('alpha = 1','alpha = 2','alpha = 4','Reference');
grid on;
nexttile;
plot(t,torque(:,3:5),'LineWidth',1.2);
ylabel('T (N m)');
xlabel('Time (s)');
legend('alpha = 1','alpha = 2','alpha = 4');
grid on;
exportgraphics(gcf,'results/task5_fast_observers.png','Resolution',200);
