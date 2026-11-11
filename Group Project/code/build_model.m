function model = build_model(plant, observer, feedback, useFeedforward)
% Copy the original nonlinear plant, expose speeds only for validation,
% and wire the same LQE in both the Step 2 and Step 3 simulations.
root = fileparts(mfilename('fullpath'));
source = 'Non_linear_simulation_r2018b';
load_system(fullfile(root,[source '.slx']));
sourceCleanup = onCleanup(@() close_system(source,0));
subsystems = find_system(source,'SearchDepth',1,'BlockType','SubSystem');
sourcePlant = subsystems(contains(subsystems,'SRV02-ET'));
assert(numel(sourcePlant)==1,'Expected one supplied nonlinear plant.');
if useFeedforward
    model = 'Step3_Feedforward';
else
    model = 'Step2_Balance';
end
if bdIsLoaded(model), close_system(model,0); end
modelFile = fullfile(root,[model '.slx']);
if isfile(modelFile)
    load_system(modelFile);
    Simulink.BlockDiagram.deleteContents(model);
else
    new_system(model);
end
set_param(model,'StopTime','32','Solver','ode45','MaxStep','0.01', ...
    'RelTol','1e-8','AbsTol','1e-10','AlgebraicLoopMsg','none', ...
    'InitFcn','initialise_model_visualiser(bdroot);', ...
    'SignalLogging','on','SignalLoggingName','logsout','ReturnWorkspaceOutputs','on');
add_block(sourcePlant{1},[model '/Nonlinear plant'],'Position',[650 90 805 205]);
add_block([source '/visualiser'],[model '/visualiser'], ...
    'Position',[1060 245 1195 300]);
add('simulink/Signal Routing/Mux','Visualiser angles',[995 240 1000 300],'Inputs','2');
wire('Nonlinear plant/1','Visualiser angles/1','');
wire('Nonlinear plant/2','Visualiser angles/2','');
wire('Visualiser angles/1','visualiser/1','');
plantBlock = [model '/Nonlinear plant'];
set_param([plantBlock '/Integrator3'],'InitialCondition','alpha0');
add_block('simulink/Sinks/Out1',[plantBlock '/theta_dot'],'Port','3', ...
    'Position',[1090 175 1120 189]);
add_block('simulink/Sinks/Out1',[plantBlock '/alpha_dot'],'Port','4', ...
    'Position',[1090 760 1120 774]);
add_line(plantBlock,'Integrator/1','theta_dot/1','autorouting','on');
add_line(plantBlock,'Integrator2/1','alpha_dot/1','autorouting','on');

add('simulink/Sources/Step','Unknown voltage disturbance',[515 35 550 65], ...
    'Time','disturbanceTime','Before','0','After','disturbanceV');
add('simulink/Math Operations/Sum','Plant voltage',[590 123 620 153], ...
    'Inputs','++');
add('simulink/Signal Routing/Mux','Observer inputs',[295 265 300 345],'Inputs','3');
add('simulink/Continuous/State-Space','Step 1 LQE',[345 280 455 330], ...
    'A','A_obs','B','B_obs','C','C_obs','D','D_obs','X0','xhat0');
add('simulink/Math Operations/Gain','State feedback',[500 280 580 330], ...
    'Gain','-Kx','Multiplication','Matrix(K*u)');
add('simulink/Signal Routing/Mux','True state for plots',[880 88 885 207],'Inputs','4');
add('simulink/Sinks/Terminator','True state log',[930 135 950 155]);

wire('Unknown voltage disturbance/1','Plant voltage/2','disturbance');
wire('Plant voltage/1','Nonlinear plant/1','');
wire('Nonlinear plant/2','True state for plots/1','');
wire('Nonlinear plant/1','True state for plots/2','');
wire('Nonlinear plant/3','True state for plots/3','');
wire('Nonlinear plant/4','True state for plots/4','');
wire('True state for plots/1','True state log/1','x_true');
wire('Nonlinear plant/2','Observer inputs/2','');
wire('Nonlinear plant/1','Observer inputs/3','');
wire('Observer inputs/1','Step 1 LQE/1','');
wire('Step 1 LQE/1','State feedback/1','x_hat');

if useFeedforward
    add('simulink/Sources/From Workspace','Arm reference',[65 104 155 151], ...
        'VariableName','referenceProfile','Interpolate','off','OutputAfterFinalValue','Holding final value');
    add('simulink/Math Operations/Gain','Reference feedforward',[285 110 375 150], ...
        'Gain','Nbar');
    add('simulink/Math Operations/Sum','Motor command',[630 285 660 320], ...
        'Inputs','++');
    add('simulink/Discontinuities/Saturation','Voltage limit',[700 280 775 325], ...
        'UpperLimit','voltageLimit','LowerLimit','-voltageLimit');
    wire('Arm reference/1','Reference feedforward/1','reference');
    wire('Reference feedforward/1','Motor command/1','');
    wire('State feedback/1','Motor command/2','');
    wire('Motor command/1','Voltage limit/1','u_requested');
    command = 'Voltage limit/1';
else
    command = 'State feedback/1';
end
wire(command,'Plant voltage/1','u');
wire(command,'Observer inputs/1','');

ws = get_param(model,'ModelWorkspace');
names = fieldnames(plant);
for index = 1:numel(names)
    assignin(ws,names{index},plant.(names{index}));
end
names = {'A_obs','B_obs','C_obs','D_obs'};
for index = 1:numel(names)
    assignin(ws,names{index},observer.(names{index}));
end
assignin(ws,'Kx',feedback.Kx);
assignin(ws,'alpha0',deg2rad(5));
assignin(ws,'xhat0',zeros(4,1));
assignin(ws,'disturbanceTime',20);
assignin(ws,'disturbanceV',0);
if useFeedforward
    assignin(ws,'Nbar',feedback.Nbar);
    assignin(ws,'voltageLimit',feedback.voltageLimit);
    assignin(ws,'referenceProfile',[0 0; 8 deg2rad(20); 16 deg2rad(-15); 24 deg2rad(30); 32 0]);
    set_param(model,'StopTime','40');
else
    set_param(model,'StopTime','8');
end
save_system(model,modelFile);

    function add(library,name,position,varargin)
        add_block(library,[model '/' name],'Position',position,varargin{:});
    end

    function wire(from,to,label)
        line = add_line(model,from,to,'autorouting','on');
        if strlength(label)>0
            set_param(line,'Name',label);
            sourcePort = get_param(line,'SrcPortHandle');
            set_param(sourcePort,'DataLogging','on');
        end
    end
end
