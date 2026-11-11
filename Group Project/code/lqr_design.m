function design = lqr_design(observer)
% LQR design function for nonlinear simulations; uses the same cases as step2.m.
if nargin==0
    [~,observer] = step1;
end
A = observer.A;
B = observer.B;
Q_values = [10 100 1 10; 10 300 1 10; 10 100 1 10; 10 100 1 10];
R_values = [1 1 0.1 10];
t = (0:0.01:5)';
x0 = [0; deg2rad(5); 0; 0];
K_all = zeros(4,4);
angles = zeros(numel(t),4);
pendulum = angles;
voltage = angles;
for index = 1:4
    K = lqr(A,B,diag(Q_values(index,:)),R_values(index));
    K_all(index,:) = K;
    closedLoop = ss(A-B*K,zeros(4,1),eye(4),zeros(4,1));
    x = initial(closedLoop,x0,t);
    angles(:,index) = rad2deg(x(:,1));
    pendulum(:,index) = rad2deg(x(:,2));
    voltage(:,index) = -x*K';
end
selected_case = 2;
K = K_all(selected_case,:);
Q = diag(Q_values(selected_case,:));
R = R_values(selected_case);
assert(all(real(eig(A-B*K))<0));
caseNumber = (1:4)';
comparison = table(caseNumber,Q_values(:,2),R_values',max(abs(angles))', ...
    max(abs(voltage))',caseNumber==selected_case, ...
    'VariableNames',{'Case','Q_alpha','R','PeakArm_deg','PeakVoltage_V','Selected'});
design = struct('Q',Q,'R',R,'K',K,'comparison',comparison,'t',t, ...
    'angles',angles,'pendulum',pendulum,'voltage',voltage);
disp(comparison);
fprintf('STEP2_OK | selected case %d\n',selected_case);
end
