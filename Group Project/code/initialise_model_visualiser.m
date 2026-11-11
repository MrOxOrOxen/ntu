function initialise_model_visualiser(model)
% Initialise the supplied visualiser for the current simulation duration.
root = fileparts(mfilename('fullpath'));
callerFolder = pwd;
restoreFolder = onCleanup(@() cd(callerFolder));
cd(root);
lengthOfReplay = str2double(get_param(model,'StopTime'));
[armPartPatchHandle,armPartVertices,pendPartPatchHandle,pendPartVertices, ...
    drivePartPatchHandle,drivePartVertices,thetaPlotHandle,alphaPlotHandle] = ...
    initialiseVisualiser(lengthOfReplay);
ws = 'base';
assignin(ws,'armPartPatchHandle',double(armPartPatchHandle));
assignin(ws,'armPartVertices',armPartVertices);
assignin(ws,'pendPartPatchHandle',double(pendPartPatchHandle));
assignin(ws,'pendPartVertices',pendPartVertices);
assignin(ws,'drivePartPatchHandle',double(drivePartPatchHandle));
assignin(ws,'drivePartVertices',drivePartVertices);
assignin(ws,'thetaPlotHandle',double(thetaPlotHandle));
assignin(ws,'alphaPlotHandle',double(alphaPlotHandle));
assignin(ws,'frameRate',50);
drawnow;
end
