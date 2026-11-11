function run_step3_required(results, plant, observer, baseline)
% Run LQR plus reference feedforward tracking with zero applied disturbance.
root = fileparts(mfilename('fullpath'));
if nargin==0, results = fullfile(root,'results_required'); end
if ~isfolder(results), mkdir(results); end
if nargin<2
    [plant,observer] = step1;
    baseline = lqr_design(observer);
end
design = step3(observer,baseline);
model = build_model(plant,observer,design,true);
% cleanup = onCleanup(@() close_system(model,0));
open_system(model);
[data,tracking] = simulate_case(model,'step3_reference_tracking',results,0);
assert(isequal(round(tracking.Reference_deg)',[0 20 -15 30 0]));
assert(all(tracking.Disturbance_V==0));
assert(max(abs(data.Command_V)) <= design.voltageLimit+1e-10);
save(fullfile(results,'step3_design.mat'),'observer','baseline','design');
fprintf('STEP3_REQUIRED_OK | switched peak tilt %.6f deg | switched tail error %.9f deg\n', ...
    max(tracking.PeakTilt_deg(2:end)),max(tracking.TailMaxError_deg(2:end)));
end
