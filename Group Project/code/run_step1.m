function [plant,observer] = run_step1(showSimulation)
% Reproduce the estimator and display it in the nonlinear balance simulation.
if nargin==0, showSimulation = true; end
[plant,observer] = step1;
root = fileparts(mfilename('fullpath'));
save(fullfile(root,'step1_design.mat'),'plant','observer');
if showSimulation
    baseline = lqr_design(observer);
    model = build_model(plant,observer,struct('Kx',baseline.K),false);
    open_system(model);
    results = fullfile(root,'results_required');
    if ~isfolder(results), mkdir(results); end
    simulate_case(model,'step1_observer',results,0);
end
end
