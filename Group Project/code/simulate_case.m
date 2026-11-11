function [data, summary] = simulate_case(model, name, folder, disturbanceV)
% The observer receives commanded voltage. The plant alone sees disturbance.
input = Simulink.SimulationInput(model);
input = input.setVariable('disturbanceV',disturbanceV,'Workspace',model);
if disturbanceV~=0
    input = input.setVariable('referenceProfile',[0 0; 8 deg2rad(20)],'Workspace',model);
end
result = sim(input);
logs = result.logsout;
truth = logs.get('x_true').Values;
[t,rows] = unique(truth.Time,'last');
x = truth.Data(rows,:);
xhat = sample(logs,'x_hat',t,'linear');
u = sample(logs,'u',t,'linear');
disturbance = sample(logs,'disturbance',t,'previous');
tracking = strcmp(model,'Step3_Feedforward');
if tracking
    reference = sample(logs,'reference',t,'previous');
else
    reference = zeros(size(t));
end
data = table(t,rad2deg(reference),rad2deg(x(:,1)),rad2deg(x(:,2)), ...
    rad2deg(reference-x(:,1)),u,disturbance,x(:,3),xhat(:,3),x(:,4),xhat(:,4), ...
    'VariableNames',{'Time_s','Reference_deg','Arm_deg','Pendulum_deg','Error_deg', ...
    'Command_V','Disturbance_V','ArmSpeed_rad_s','EstimatedArmSpeed_rad_s', ...
    'PendulumSpeed_rad_s','EstimatedPendulumSpeed_rad_s'});
event = [true; abs(diff(reference))>1e-8 | abs(diff(disturbance))>1e-8];
start = find(event);
stop = [start(2:end)-1; numel(t)];
n = numel(start);
tailError = zeros(n,1);
tailTilt = zeros(n,1);
peakTilt = zeros(n,1);
peakVoltage = zeros(n,1);
peakError = zeros(n,1);
for index = 1:n
    interval = start(index):stop(index);
    tail = interval(t(interval)>=t(stop(index))-1);
    tailError(index) = max(abs(data.Error_deg(tail)));
    tailTilt(index) = max(abs(data.Pendulum_deg(tail)));
    peakTilt(index) = max(abs(data.Pendulum_deg(interval)));
    peakVoltage(index) = max(abs(u(interval)));
    peakError(index) = max(abs(data.Error_deg(interval)));
end
summary = table(t(start),data.Reference_deg(start),disturbance(start), ...
    tailError,tailTilt,peakTilt,peakVoltage,peakError,'VariableNames', ...
    {'Start_s','Reference_deg','Disturbance_V','TailMaxError_deg', ...
    'TailMaxTilt_deg','PeakTilt_deg','PeakCommand_V','PeakTrackingError_deg'});
disp(summary);
assert(all(tailError<0.1) && all(tailTilt<0.1) && all(peakTilt<15), ...
    'The selected nonlinear simulation did not meet the project self-check limits.');
if ~tracking
    final = t>=t(end)-1;
    assert(max(abs(x(final,3:4)-xhat(final,3:4)),[],'all')<0.01, ...
        'The baseline velocity estimate did not converge.');
end
fprintf('CASE_OK | %s | peak tilt %.4f deg | peak command %.4f V\n', ...
    name,max(peakTilt),max(peakVoltage));
writetable(data,fullfile(folder,[name '_samples.csv']));
writetable(summary,fullfile(folder,[name '_summary.csv']));
save(fullfile(folder,[name '.mat']),'data','summary');

fig = figure('Visible','off','Color','w','Position',[100 100 1150 900]);
cleanup = onCleanup(@() close(fig));
tiles = tiledlayout(fig,4,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(tiles);
stairs(ax,t,data.Reference_deg,'k--','LineWidth',1.3); hold(ax,'on');
plot(ax,t,data.Arm_deg,'b','LineWidth',1.5); ylabel(ax,'Arm (deg)');
legend(ax,'Reference','Actual','Location','best'); grid(ax,'on');
ax = nexttile(tiles);
plot(ax,t,data.Pendulum_deg,'Color',[.1 .48 .23],'LineWidth',1.5);
ylabel(ax,'Pendulum (deg)'); grid(ax,'on');
ax = nexttile(tiles);
plot(ax,t,data.Error_deg,'LineWidth',1.5);
ylabel(ax,'Tracking error (deg)'); grid(ax,'on');
ax = nexttile(tiles);
plot(ax,t,u,'LineWidth',1.5);
ylabel(ax,'Motor voltage (V)'); xlabel(ax,'Time (s)'); grid(ax,'on');
if ~tracking || any(disturbance~=0)
    hold(ax,'on');
    stairs(ax,t,disturbance,'--','LineWidth',1.3);
    legend(ax,'Motor command','Unknown disturbance','Location','best');
end
title(tiles,strrep(name,'_',' '));
exportgraphics(fig,fullfile(folder,[name '.png']),'Resolution',160);
savefig(fig,fullfile(folder,[name '.fig']));
end

function y = sample(logs,name,t,method)
series = logs.get(name).Values;
[time,rows] = unique(series.Time,'last');
values = series.Data(rows,:);
y = interp1(time,values,t,method,'extrap');
end
