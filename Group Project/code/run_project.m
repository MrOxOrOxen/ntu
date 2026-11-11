function run_project

projectRoot = fileparts(mfilename('fullpath'));
results = fullfile(projectRoot,'results_required');

callerFolder = pwd;
restoreFolder = onCleanup(@() cd(callerFolder));
cd(projectRoot);
if ~isfolder(results), mkdir(results); end
fprintf('MATLAB %s\nProject root: %s\nResults folder: %s\n',version,projectRoot,results);
[plant,observer] = run_step1;
baseline = run_step2(results,plant,observer);
run_step3_required(results,plant,observer,baseline);
fprintf('PROJECT_COMPLETE\n');
end
