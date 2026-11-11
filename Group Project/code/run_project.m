function run_project
% Run all three stages from the 6217 folder.
% Individual entries: MA6217_intial, oberver_performance, step2,
% run_step2 (nonlinear balancing), run_step3_required (feedforward tracking).
% lqr_design is the function used by the nonlinear simulation entries.
% Results are generated in results_required when this entry is run.
projectRoot = fileparts(mfilename('fullpath'));
results = fullfile(projectRoot,'results_required');
% Simulink's default generated files follow the current MATLAB folder.
callerFolder = pwd;
restoreFolder = onCleanup(@() cd(callerFolder));
cd(projectRoot);
if ~isfolder(results), mkdir(results); end
fprintf('MATLAB %s\nProject root: %s\nResults folder: %s\n',version,projectRoot,results);
[plant,observer] = run_step1(false);
baseline = run_step2(results,plant,observer);
run_step3_required(results,plant,observer,baseline);
fprintf('PROJECT_COMPLETE\n');
end
