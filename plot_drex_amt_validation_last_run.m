function [fig, ax, data] = plot_drex_amt_validation_last_run(varargin)
%PLOT_DREX_AMT_VALIDATION_LAST_RUN Compare D-REX AMT with its measured profile.
%   plot_drex_amt_validation_last_run uses the latest SDI run (or workspace
%   out when SDI has no runs).
%   plot_drex_amt_validation_last_run('Rerun',true) captures actual motor speeds
%   in a new run, without replacing out or saving/changing model parameters.
%   [fig,ax,data] = plot_drex_amt_validation_last_run('XLim',[0 6], ...
%       'TorqueLim',[-80 150], 'SpeedUnit','rpm');
%   Options: XLim, TorqueLim, SpeedLim, VehicleSpeedLim, SpeedUnit ('rpm' or
%   'rad/s'), Rerun, RunID, SimulationOutput, Profile, MeasuredTorque ('set'
%   or 'actual'). RunID selects an SDI run; SimulationOutput selects a supplied
%   Simulink.SimulationOutput. Default Profile is current_profile in base.
%   The profile must match the selected simulation; original timestamps apply.
%   EM1 speed is taken from the input of the 'simulated EM1 - speed' scope
%   inside D-REX_AMT/Clutch (rad/s), rather than the engine speed selector.
%   EM1 is the shifting motor (model M_engine/M_MCU2); EM2 is the fixed-ratio
%   motor (model M_EM1/M_MCU1). Returned motor speeds are always in rpm.
%   Motor-speed logging remains enabled for subsequent ordinary simulations.
%   If the latest run lacks actual speeds, a fresh run is captured. Explicit
%   RunID/SimulationOutput selections are never silently replaced.
%   exportgraphics(fig,'drex_amt_validation.png','Resolution',300)

p = inputParser;
addParameter(p,'XLim',[0 6],@validLimits);
addParameter(p,'TorqueLim',[-50 150],@validLimits);
addParameter(p,'SpeedLim',[],@validLimits);
addParameter(p,'VehicleSpeedLim',[],@validLimits);
addParameter(p,'SpeedUnit','rpm',@(x) any(strcmpi(x,{'rpm','rad/s'})));
addParameter(p,'Rerun',false,@(x) islogical(x) && isscalar(x));
addParameter(p,'RunID',[],@(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && isfinite(x) && x>0));
addParameter(p,'SimulationOutput',[],@(x) isempty(x) || isa(x,'Simulink.SimulationOutput'));
addParameter(p,'Profile',[],@(x) isempty(x) || isstruct(x));
addParameter(p,'MeasuredTorque','set',@(x) any(strcmpi(x,{'set','actual'})));
parse(p,varargin{:});
opt = p.Results;
assert(sum([opt.Rerun ~isempty(opt.RunID) ~isempty(opt.SimulationOutput)])<=1, ...
    'Choose only one of Rerun, RunID, or SimulationOutput.');
profile = opt.Profile;
if isempty(profile)
    assert(evalin('base','exist(''current_profile'',''var'')==1'), ...
        'Load the matching current_profile, or supply Profile.');
    profile = evalin('base','current_profile');
end
assert(isfield(profile,'data'),'Profile must contain data.');
model = 'simulation_upgrade';
engine = [model '/DRIVETRAIN/Engine/D-REX_AMT_Engine'];
clutch = [model '/DRIVETRAIN/Transmission/Gearbox/D-REX_AMT/Clutch'];
scope = [clutch '/simulated EM1 - speed'];
em1Source = [clutch '/Switch'];
if bdIsLoaded(model)
    % Follow the user's scope wiring, including future changes to its source.
    assert(getSimulinkBlockHandle(scope)>0,'Missing EM1 speed scope: %s',scope);
    ph = get_param(scope,'PortHandles');
    assert(numel(ph.Inport)==1,'EM1 speed scope must have one input.');
    line = get_param(ph.Inport(1),'Line');
    assert(line~=-1,'Connect the EM1 speed scope before plotting.');
    src = get_param(line,'SrcPortHandle');
    em1Source = get_param(src,'Parent');
    em1Port = get_param(src,'PortNumber');
else
    em1Port = 1;
end
speedPaths = {em1Source,[engine '/Saturation4']};
speedPorts = [em1Port 1];
if bdIsLoaded(model) && strcmp(get_param(model,'SimulationStatus'),'stopped')
    for k = 1:numel(speedPaths)
        Simulink.sdi.markSignalForStreaming(speedPaths{k},speedPorts(k),'on');
    end
end
if opt.Rerun
    assert(bdIsLoaded(model),'Open simulation_upgrade before capturing a run.');
    assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
        'Wait for the simulation to stop.');
    % Instrument only the signals needed here; restore their previous flags.
    paths = [{[model '/PANEL/From'],[model '/PANEL/From12'], ...
        [model '/PANEL/From13']} speedPaths];
    ports = [1 1 1 speedPorts];
    old = cell(size(paths));
    for k = 1:numel(paths)
        ph = get_param(paths{k},'PortHandles');
        old{k} = get_param(ph.Outport(ports(k)),'DataLogging');
    end
    dirty = get_param(model,'Dirty');
    cleanup = onCleanup(@() restoreLogging(paths,ports,old,model,dirty));
    for k = 1:numel(paths)
        Simulink.sdi.markSignalForStreaming(paths{k},ports(k),'on');
    end
    logging = Simulink.SimulationData.ModelLoggingInfo.createFromModel(model);
    logging.LoggingMode = 'LogAllAsSpecifiedInModel';
    in = Simulink.SimulationInput(model);
    in = in.setModelParameter('SignalLogging','on', ...
        'SignalLoggingName','logsout','DataLoggingOverride',logging);
    out = sim(in);
    ds = out.get('logsout');
    source = 'New D-REX AMT capture';
    clear cleanup
elseif ~isempty(opt.RunID) || (isempty(opt.SimulationOutput) ...
        && ~isempty(Simulink.sdi.getAllRunIDs))
    runID = opt.RunID;
    if isempty(runID)
        ids = Simulink.sdi.getAllRunIDs;
        runID = ids(end);
    end
    run = Simulink.sdi.getRun(runID);
    ds = run.export;
    source = sprintf('SDI run %g: %s',runID,run.Name);
else
    out = opt.SimulationOutput;
    if isempty(out)
        assert(evalin('base','exist(''out'',''var'')==1'), ...
            'No workspace out. Supply SimulationOutput, RunID, or Rerun=true.');
        out = evalin('base','out');
    end
    assert(isa(out,'Simulink.SimulationOutput'),'out must be a SimulationOutput.');
    assert(strcmp(out.SimulationMetadata.ModelInfo.ModelName,model), ...
        'Selected output is not from simulation_upgrade.');
    ds = out.get('logsout');
    source = 'Workspace simulation output';
end

vehicle = readSeries(ds,[model '/PANEL/From'],1,'v_fdbk');
% Bus leaves are selected by name, never by their incidental ordering.
data.vehicle_speed_sim = vehicle;
data.motor1_torque_sim = readSeries(ds,[model '/PANEL/From12'],1,'M_engine');
data.motor2_torque_sim = readSeries(ds,[model '/PANEL/From12'],1,'M_EM1');
data.motor1_torque_command = readSeries(ds,[model '/PANEL/From13'],1,'M_MCU2');
data.motor2_torque_command = readSeries(ds,[model '/PANEL/From13'],1,'M_MCU1');
data.motor1_speed_sim = readSeries(ds,speedPaths{1},speedPorts(1),'');
data.motor2_speed_sim = readSeries(ds,speedPaths{2},speedPorts(2),'');
data.motor1_speed_source = em1Source;
if (isempty(data.motor1_speed_sim) || isempty(data.motor2_speed_sim)) ...
        && ~opt.Rerun && isempty(opt.RunID) && isempty(opt.SimulationOutput)
    fprintf('Latest run lacks actual motor speeds; capturing them in a new run.\n');
    opt.Rerun = true;
    opt.Profile = profile;
    [fig,ax,data] = plot_drex_amt_validation_last_run(opt);
    return
end
assert(~isempty(vehicle) && ~isempty(data.motor1_torque_sim) ...
    && ~isempty(data.motor2_torque_sim), ...
    'Selected run lacks D-REX PANEL signals. Use Rerun=true.');
if ~isempty(data.motor1_speed_sim)
    data.motor1_speed_sim = timeseries(data.motor1_speed_sim.Data*60/(2*pi), ...
        data.motor1_speed_sim.Time);
else
    warning('drex:MissingSpeed', ...
        'Actual EM1 speed was not logged. Use Rerun=true to include it.');
end
if isempty(data.motor2_speed_sim)
    warning('drex:MissingSpeed','EM2 speed was not logged. Use Rerun=true.');
end
suffix = 'set';
if strcmpi(opt.MeasuredTorque,'actual'), suffix = 'act'; end
measurementFields = {'veh_speed',['EM1_trq_' suffix], ...
    ['EM2_trq_' suffix],'EM1_speed','EM2_speed'};
measurementNames = {'vehicle_speed_measured','motor1_torque_measured', ...
    'motor2_torque_measured','motor1_speed_measured','motor2_speed_measured'};
for k = 1:numel(measurementFields)
    assert(isfield(profile.data,measurementFields{k}) && isfield(profile.data,'time'), ...
        'Profile lacks %s or time.',measurementFields{k});
    data.(measurementNames{k}) = timeseries( ...
        profile.data.(measurementFields{k})(:),profile.data.time(:));
end
data.source = source;
data.profile_meta = [];
if isfield(profile,'meta'), data.profile_meta = profile.meta; end
data.measured_torque_type = opt.MeasuredTorque;

fig = figure('Color','w','Name','D-REX AMT experimental validation', ...
    'Position',[100 60 1100 900]);
layout = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
ax = gobjects(3,1);
for k = 1:3, ax(k) = nexttile(layout); end
blue = [0 .36 .65]; orange = [.85 .33 .10];
drawSeries(ax(1),data.vehicle_speed_sim,blue,'-','Simulated speed',1);
drawSeries(ax(1),data.vehicle_speed_measured,orange,'--','Measured speed',1);
factor = 1;
if strcmpi(opt.SpeedUnit,'rad/s'), factor = 2*pi/60; end
for motor = 1:2
    colors = [blue; orange];
    color = colors(motor,:);
    prefix = sprintf('motor%d_',motor);
    drawSeries(ax(2),data.([prefix 'torque_sim']),color,'-', ...
        sprintf('Motor %d simulated torque',motor),1);
    drawSeries(ax(2),data.([prefix 'torque_measured']),color,'--', ...
        sprintf('Motor %d measured torque',motor),1);
    drawSeries(ax(3),data.([prefix 'speed_sim']),color,'-', ...
        sprintf('Motor %d simulated rotation',motor),factor);
    drawSeries(ax(3),data.([prefix 'speed_measured']),color,'--', ...
        sprintf('Motor %d measured rotation',motor),factor);
end
for k = 1:3
    grid(ax(k),'on');
    set(ax(k),'FontName','Arial','FontSize',11,'Box','off', ...
        'TickDir','out','GridAlpha',.15,'XMinorTick','on');
    legend(ax(k),'Location','northwest','Box','on');
end
ylabel(ax(1),'Vehicle speed (km/h)');
ylabel(ax(2),'Torque (N m)');
ylabel(ax(3),sprintf('Motor speed (%s)',lower(opt.SpeedUnit)));
xlabel(ax(3),'Time (s)');
ax(1).XTickLabel = []; ax(2).XTickLabel = [];
linkaxes(ax,'x');
if ~isempty(opt.XLim), xlim(ax(1),opt.XLim); end
if ~isempty(opt.TorqueLim), ylim(ax(2),opt.TorqueLim); end
if ~isempty(opt.SpeedLim), ylim(ax(3),opt.SpeedLim); end
if ~isempty(opt.VehicleSpeedLim), ylim(ax(1),opt.VehicleSpeedLim); end
end

function ts = readSeries(ds,path,port,leaf)
ts = [];
for k = 1:ds.numElements
    signal = ds.getElement(k);
    if ~strcmp(signal.BlockPath.getBlock(1),path) || signal.PortIndex~=port
        continue
    end
    value = signal.Values;
    if isstruct(value) && ~isempty(leaf) && isfield(value,leaf)
        ts = value.(leaf); return
    elseif isa(value,'timeseries') && (isempty(leaf) || ...
            strcmp(signal.Name,leaf) || endsWith(signal.Name,['.' leaf]))
        ts = value; return
    end
end
end

function drawSeries(ax,ts,color,style,label,factor)
if isempty(ts), return; end
y = squeeze(ts.Data);
assert(isvector(y) && numel(y)==numel(ts.Time),'%s must be scalar.',label);
hold(ax,'on');
plot(ax,ts.Time,y*factor,'Color',color,'LineStyle',style, ...
    'LineWidth',1.7,'DisplayName',label);
end

function tf = validLimits(x)
tf = isempty(x) || (isnumeric(x) && isreal(x) && numel(x)==2 ...
    && all(isfinite(x)) && x(1)<x(2));
end

function restoreLogging(paths,ports,old,model,dirty)
for k = 1:numel(paths)
    Simulink.sdi.markSignalForStreaming(paths{k},ports(k),old{k});
end
% Restoring logging flags is the only persistent edit made by this function.
set_param(model,'Dirty',dirty);
end
