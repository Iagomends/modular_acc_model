function [fig, ax, data] = plot_tdt_validation_last_run(varargin)
%PLOT_TDT_VALIDATION_LAST_RUN Plot the three PANEL validation comparisons.
%   plot_tdt_validation_last_run reads the latest SDI run.
%   Enables validation signal logging for subsequent ordinary simulations.
%   If the latest run lacks these signals, automatically captures a new run.
%   plot_tdt_validation_last_run('Rerun',true) captures a new simulation.
%   [fig,ax,data] = plot_tdt_validation_last_run('XLim',[2 4], ...
%       'TorqueLim',[-120 150], 'SpeedLim',[0 3500], ...
%       'VehicleSpeedLim',[0 40], 'SpeedUnit','rpm');
%   Options: XLim (default []), TorqueLim, SpeedLim (motor speed),
%   VehicleSpeedLim, SpeedUnit ('rpm' or 'rad/s'), Rerun, RunID.
%   Empty limits select automatic scaling. All three time axes are linked.
%   Returned data contains captured timeseries in the PANEL source units:
%   vehicle speed km/h, torque N*m, both motor speeds rpm. SpeedUnit changes only
%   the plotted motor speed. Measured torques are the PANEL setpoint traces.
%   MCU commands are returned in data but omitted from the figure.
%   For saved runs, keep current_profile set to the experiment used by that
%   run: measured motor 2 speed comes from this profile, not an SDI signal.
%   exportgraphics(fig,'tdt_validation.png','Resolution',300)

p = inputParser;
addParameter(p,'XLim',[0 6],@validLimits);
addParameter(p,'TorqueLim',[-50 150],@validLimits);
addParameter(p,'SpeedLim',[],@validLimits);
addParameter(p,'VehicleSpeedLim',[],@validLimits);
addParameter(p,'SpeedUnit','rpm',@(x) any(strcmpi(x,{'rad/s','rpm'})));
addParameter(p,'Rerun',false,@(x) islogical(x) && isscalar(x));
addParameter(p,'RunID',[],@(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && isfinite(x) && x>0));
parse(p,varargin{:});
opt = p.Results;
model = 'simulation_upgrade';
assert(bdIsLoaded(model),'Open simulation_upgrade before plotting validation.');
% Resolve the actual scope input sources, preserving the current PANEL wiring.
scopeIDs = [8931 8940 8977];
paths = cell(1,8);
ports = zeros(1,8);
n = 0;
for k = 1:3
    scope = Simulink.ID.getFullName(sprintf('%s:%d',model,scopeIDs(k)));
    ph = get_param(scope,'PortHandles');
    for j = 1:numel(ph.Inport)
        line = get_param(ph.Inport(j),'Line');
        assert(line~=-1,'Validation scope has an unconnected input.');
        src = get_param(line,'SrcPortHandle');
        n = n+1;
        paths{n} = get_param(src,'Parent');
        ports(n) = get_param(src,'PortNumber');
    end
end
assert(n==8,'Expected PANEL validation scopes with 2, 4, and 2 inputs.');
% The PANEL speed scope combines the motor 2 path with measured EM1 speed.
% Also capture the motor 1 speed used by its torque-speed characteristic.
n = n+1;
paths{n} = Simulink.ID.getFullName(sprintf('%s:3480',model));
ports(n) = 1;

% Keep instrumentation enabled so the Simulink Run button records the same
% signals as Rerun. Do not restore the old logging flags after capture.
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
    'Wait for the current simulation to finish before plotting validation.');
for k = 1:n
    ph = get_param(paths{k},'PortHandles');
    set_param(ph.Outport(ports(k)),'DataLogging','on');
end
logging = Simulink.SimulationData.ModelLoggingInfo.createFromModel(model);
logging.LoggingMode = 'LogAllAsSpecifiedInModel';
set_param(model,'SignalLogging','on','SignalLoggingName','logsout', ...
    'DataLoggingOverride',logging);

if ~opt.Rerun
    ids = Simulink.sdi.getAllRunIDs;
    if isempty(opt.RunID)
        if isempty(ids)
            opt.Rerun = true;
        else
            run = Simulink.sdi.getRun(ids(end));
            if ~hasValidationSignals(run,paths,ports)
                fprintf('Latest run lacks validation signals; capturing a fresh simulation.\n');
                opt.Rerun = true;
            end
        end
    else
        % An explicit historical run must never be silently replaced.
        run = Simulink.sdi.getRun(opt.RunID);
        assert(hasValidationSignals(run,paths,ports), ...
            'Selected RunID lacks validation signals. Omit RunID to capture the latest data.');
    end
end

values = cell(1,n);
if opt.Rerun
    assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
        'Stop the current simulation before capturing another run.');
    in = Simulink.SimulationInput(model);
    in = in.setModelParameter('SignalLogging','on', ...
        'SignalLoggingName','logsout','DataLoggingOverride',logging);
    out = sim(in);
    ds = out.get('logsout');
    for k = 1:n
        matches = [];
        for j = 1:ds.numElements
            sig = ds.getElement(j);
            if strcmp(sig.BlockPath.getBlock(1),paths{k}) && sig.PortIndex==ports(k)
                matches(end+1) = j; %#ok<AGROW>
            end
        end
        assert(isscalar(matches),'Could not uniquely capture %s port %d.',paths{k},ports(k));
        sig = ds.getElement(matches);
        values{k} = sig.Values;
    end
else
    for k = 1:n
        matches = [];
        for j = 1:run.SignalCount
            sig = run.getSignalByIndex(j);
            if strcmp(sig.BlockPath,paths{k}) && sig.PortIndex==ports(k)
                matches(end+1) = j; %#ok<AGROW>
            end
        end
        assert(~isempty(matches), ...
            'Run "%s" lacks PANEL input %s port %d. Use plot_tdt_validation_last_run(''Rerun'',true).', ...
            run.Name,paths{k},ports(k));
        % SDI exposes bus leaves as separate signals; keep their source order.
        leaves = {};
        for j = matches
            sig = run.getSignalByIndex(j);
            if k==1 && ~endsWith(sig.Name,'.v_fdbk')
                continue
            end
            if sig.NumPoints>0 && isa(sig.Values,'timeseries')
                leaves{end+1} = sig.Values; %#ok<AGROW>
            end
        end
        values{k} = leaves;
    end
end

names = {'vehicle_speed_sim','vehicle_speed_measured', ...
    'motor1_torque_sim','motor2_torque_sim', ...
    'motor1_torque_command','motor2_torque_command', ...
    'motor1_torque_measured','motor2_torque_measured', ...
    'motor2_speed_sim','motor1_speed_measured','motor1_speed_sim'};
series = {};
expected = [1 1 2 2 1 1 1 1 1];
for k = 1:n
    if k==1 && isstruct(values{k})
        % v_control also contains speed error and target; compare feedback.
        leaves = flattenSeries(values{k}.v_fdbk);
    elseif k==3 && isstruct(values{k})
        leaves = {values{k}.M_EM1, values{k}.M_EM2};
    elseif k==4 && isstruct(values{k})
        leaves = {values{k}.M_MCU1, values{k}.M_MCU2};
    else
        leaves = flattenSeries(values{k});
    end
    assert(numel(leaves)==expected(k), ...
        'PANEL input %d has %d scalar traces; expected %d.',k,numel(leaves),expected(k));
    series = [series leaves]; %#ok<AGROW>
end
data = cell2struct(series,names,2);
% PANEL has no EM2 measurement lookup. Use the loaded experimental profile,
% with its original timestamps (the same time base as the PANEL lookups).
assert(evalin('base','exist(''current_profile'',''var'')==1'), ...
    'Load current_profile for measured motor 2 speed.');
profile = evalin('base','current_profile');
assert(isfield(profile,'data') && isfield(profile.data,'EM2_speed') ...
    && isfield(profile.data,'time'),'current_profile lacks measured EM2 speed/time.');
data.motor2_speed_measured = timeseries(profile.data.EM2_speed(:),profile.data.time(:));
series{end+1} = data.motor2_speed_measured;
names{end+1} = 'motor2_speed_measured';
fig = figure('Color','w','Name','TDT experimental validation', ...
    'Position',[100 60 1100 900]);
layout = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
ax = gobjects(3,1);
for k = 1:3, ax(k) = nexttile(layout); end
panels = [1 1 2 2 2 2 2 2 3 3 3 3];
colors = [0 .36 .65; .85 .33 .10];
colorIndex = [1 2 1 2 1 2 1 2 2 1 1 2];
styles = {'-','--','-','-','--','--',':',':','-','--','-','--'};
labels = {'Simulated speed','Measured speed','Motor 1 simulated torque','Motor 2 simulated torque', '', '', ...
    'Motor 1 measured torque', 'Motor 2 measured torque', ...
    'Motor 2 simulated rotation','Motor 1 measured rotation', ...
    'Motor 1 simulated rotation','Motor 2 measured rotation'};
for k = [1:4 7:12]
    ts = series{k};
    y = squeeze(ts.Data);
    assert(isvector(y) && numel(y)==numel(ts.Time),'%s must be a scalar time series.',names{k});
    if panels(k)==3 && strcmpi(opt.SpeedUnit,'rad/s'), y = y*2*pi/60; end
    hold(ax(panels(k)),'on');
    plot(ax(panels(k)),ts.Time,y,'LineStyle',styles{k}, ...
        'Color',colors(colorIndex(k),:),'LineWidth',1.7,'DisplayName',labels{k});
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
ax(1).XTickLabel = [];
ax(2).XTickLabel = [];
linkaxes(ax,'x');
if ~isempty(opt.XLim), xlim(ax(1),opt.XLim); end
if ~isempty(opt.VehicleSpeedLim), ylim(ax(1),opt.VehicleSpeedLim); end
if ~isempty(opt.TorqueLim), ylim(ax(2),opt.TorqueLim); end
if ~isempty(opt.SpeedLim), ylim(ax(3),opt.SpeedLim); end
end

function leaves = flattenSeries(value)
leaves = {};
if isa(value,'timeseries')
    leaves = {value};
elseif iscell(value)
    for k = 1:numel(value), leaves = [leaves flattenSeries(value{k})]; end %#ok<AGROW>
elseif isstruct(value)
    fields = fieldnames(value);
    for k = 1:numel(fields), leaves = [leaves flattenSeries(value.(fields{k}))]; end %#ok<AGROW>
else
    error('Unsupported logged PANEL data type: %s.',class(value));
end
end

function tf = validLimits(x)
tf = isempty(x) || (isnumeric(x) && isreal(x) && numel(x)==2 ...
    && all(isfinite(x)) && x(1)<x(2));
end

function tf = hasValidationSignals(run,paths,ports)
counts = zeros(size(ports));
for j = 1:run.SignalCount
    sig = run.getSignalByIndex(j);
    for k = 1:numel(paths)
        if strcmp(sig.BlockPath,paths{k}) && sig.PortIndex==ports(k) ...
                && sig.NumPoints>0 && isa(sig.Values,'timeseries')
            if k~=1 || endsWith(sig.Name,'.v_fdbk')
                counts(k) = counts(k)+1;
            end
        end
    end
end
tf = isequal(counts,[1 1 2 2 1 1 1 1 1]);
end
