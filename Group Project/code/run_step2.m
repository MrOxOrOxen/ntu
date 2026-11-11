function baseline = run_step2(results, plant, observer)
root = fileparts(mfilename('fullpath'));
if nargin==0, results = fullfile(root,'results'); end
if ~isfolder(results), mkdir(results); end
if nargin<2, [plant,observer] = step1; end
baseline = lqr_design(observer);
model = build_model(plant,observer,struct('Kx',baseline.K),false);
% cleanup = onCleanup(@() close_system(model,0));
open_system(model);
data = simulate_case(model,'step2_balance',results,0);
writetable(baseline.comparison,fullfile(results,'step2_lqr_comparison.csv'));
save(fullfile(results,'step2_design.mat'),'plant','observer','baseline');

fig = figure('Visible','off','Color','w','Position',[100 100 1050 610]);
figureCleanup = onCleanup(@() close(fig));
tiles = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(tiles);
plot(ax,data.Time_s,data.ArmSpeed_rad_s,'b','LineWidth',1.5); hold(ax,'on');
plot(ax,data.Time_s,data.EstimatedArmSpeed_rad_s,'r--','LineWidth',1.2);
ylabel(ax,'Arm speed (rad/s)'); legend(ax,'True','LQE estimate'); grid(ax,'on');
ax = nexttile(tiles);
plot(ax,data.Time_s,data.PendulumSpeed_rad_s,'b','LineWidth',1.5); hold(ax,'on');
plot(ax,data.Time_s,data.EstimatedPendulumSpeed_rad_s,'r--','LineWidth',1.2);
ylabel(ax,'Pendulum speed (rad/s)'); xlabel(ax,'Time (s)'); grid(ax,'on');
title(tiles,'Step 1 LQE in Step 2 nonlinear closed loop');
exportgraphics(fig,fullfile(results,'step1_velocity_estimation.png'),'Resolution',160);
savefig(fig,fullfile(results,'step1_velocity_estimation.fig'));
fprintf('STEP2_BASELINE_OK\n');
end
