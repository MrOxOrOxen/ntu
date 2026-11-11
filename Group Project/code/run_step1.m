function [plant,observer] = run_step1
% Reproduce the estimator design and save its matrices.
[plant,observer] = step1;
root = fileparts(mfilename('fullpath'));
save(fullfile(root,'step1_design.mat'),'plant','observer');
end
