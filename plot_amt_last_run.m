function [fig, ax, data] = plot_amt_last_run(varargin)
%PLOT_AMT_LAST_RUN Plot AMT command/actual torques and synchronization speeds.
%   plot_amt_last_run reads the latest Simulation Data Inspector run.
%   plot_amt_last_run('Rerun',true) captures the five signals in a new run.
%   [fig,ax,data] = plot_amt_last_run('XLim',[3 5], ...
%       'TorqueLim',[-350 500], 'SpeedLim',[0 250], 'SpeedUnit','rad/s');
%   Use [] for automatic limits. Zoom/pan in time is linked across panels.
%   data contains the original timeseries, in N*m and rad/s.
%   exportgraphics(fig,'amt_results.png','Resolution',300)

p = inputParser;
addParameter(p,'XLim',[0 3],@validLimits);
addParameter(p,'TorqueLim',[],@validLimits);
addParameter(p,'SpeedLim',[],@validLimits);
addParameter(p,'SpeedUnit','rad/s',@(x) any(strcmpi(x,{'rad/s','rpm'})));
addParameter(p,'Rerun',false,@(x) islogical(x) && isscalar(x));
addParameter(p,'RunID',[],@(x) isempty(x) || (isnumeric(x) && isscalar(x)));
parse(p,varargin{:});
opt = p.Results;
model = 'simulation_upgrade';
names = {'M_MCU','M_engine','w_clutch','w_spur11','w_spur21'};
% Exact sources in this model; engine torque is after actuator dynamics.
sids = [4379 4019 3955 4693 4730];
ports = [1 1 2 2 2];

if opt.Rerun
    assert(bdIsLoaded(model), 'Open simulation_upgrade before capturing a run.');
    assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
        'Stop the current simulation before capturing another run.');
    oldDirty = get_param(model,'Dirty');
    handles = zeros(1,numel(names));
    old = cell(1,numel(names));
    for k = 1:numel(names)
        path = Simulink.ID.getFullName(sprintf('%s:%d',model,sids(k)));
        ph = get_param(path,'PortHandles');
        handles(k) = ph.Outport(ports(k));
        old{k} = get_param(handles(k),'DataLogging');
    end
    cleanup = onCleanup(@() restoreLogging(handles,old,model,oldDirty));
    for k = 1:numel(names)
        set_param(handles(k),'DataLogging','on');
    end
    logging = Simulink.SimulationData.ModelLoggingInfo.createFromModel(model);
    logging.LoggingMode = 'LogAllAsSpecifiedInModel';
    in = Simulink.SimulationInput(model);
    in = in.setModelParameter('SignalLogging','on', ...
        'SignalLoggingName','amtPlotLogs', ...
        'DataLoggingOverride',logging);
    out = sim(in);
    ds = out.get('amtPlotLogs');
    clear cleanup
    data = struct;
    for k = 1:numel(names)
        path = Simulink.ID.getFullName(sprintf('%s:%d',model,sids(k)));
        matches = [];
        for j = 1:ds.numElements
            sig = ds.getElement(j);
            if strcmp(sig.BlockPath.getBlock(1),path) && sig.PortIndex == ports(k)
                matches(end+1) = j; %#ok<AGROW>
            end
        end
        assert(isscalar(matches),'Could not uniquely locate captured %s.',names{k});
        sig = ds.getElement(matches);
        data.(names{k}) = sig.Values;
    end
else
    ids = Simulink.sdi.getAllRunIDs;
    assert(~isempty(ids),'No saved runs. Use plot_amt_last_run(''Rerun'',true).');
    if isempty(opt.RunID), runID = ids(end); else, runID = opt.RunID; end
    run = Simulink.sdi.getRun(runID);
    data = struct;
    missing = {};
    for k = 1:numel(names)
        path = Simulink.ID.getFullName(sprintf('%s:%d',model,sids(k)));
        matches = [];
        for j = 1:run.SignalCount
            sig = run.getSignalByIndex(j);
            if strcmp(sig.BlockPath,path) && sig.PortIndex == ports(k)
                matches(end+1) = j; %#ok<AGROW>
            end
        end
        if isscalar(matches)
            sig = run.getSignalByIndex(matches);
            data.(names{k}) = sig.Values;
        else
            missing{end+1} = names{k}; %#ok<AGROW>
        end
    end
    assert(isempty(missing), 'Run "%s" lacks: %s. Capture with plot_amt_last_run(''Rerun'',true).', ...
        run.Name,strjoin(missing,', '));
end

fig = figure('Color','w','Name','AMT synchronization', ...
    'Position',[100 100 1100 760]);
layout = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax = gobjects(2,1);
ax(1) = nexttile(layout);
ax(2) = nexttile(layout);
colors = [0.00 0.36 0.65; 0.85 0.33 0.10; 0.15 0.55 0.35];
labels = {'MCU command','Engine','Clutch','Spur 11','Spur 21'};
styles = {'--','-','-','--','--'};
seriesColors = colors([1 1 3 1 2],:);
factor = 1;
if strcmpi(opt.SpeedUnit,'rpm'), factor = 60/(2*pi); end
for k = 1:numel(names)
    panel = 1 + (k>2);
    ts = data.(names{k});
    y = squeeze(ts.Data);
    assert(isvector(y) && numel(y)==numel(ts.Time), ...
        '%s must be a scalar time series.',names{k});
    if panel==2, y = y*factor; end
    hold(ax(panel),'on');
    plot(ax(panel),ts.Time,y,'LineStyle',styles{k}, ...
        'Color',seriesColors(k,:),'LineWidth',1.7,'DisplayName',labels{k});
end
for k = 1:2
    grid(ax(k),'on');
    set(ax(k),'FontName','Arial','FontSize',11,'Box','off', ...
        'TickDir','out','GridAlpha',0.15,'XMinorTick','on');
    legend(ax(k),'Location','northeast','Orientation','vertical','Box','on');
end
ylabel(ax(1),'Torque (N m)');
ylabel(ax(2),sprintf('Angular speed (%s)',lower(opt.SpeedUnit)));
xlabel(ax(2),'Time (s)');
ax(1).XTickLabel = [];
linkaxes(ax,'x');
if ~isempty(opt.XLim), xlim(ax(1),opt.XLim); end
if ~isempty(opt.TorqueLim), ylim(ax(1),opt.TorqueLim); end
if ~isempty(opt.SpeedLim), ylim(ax(2),opt.SpeedLim); end
end

function tf = validLimits(x)
tf = isempty(x) || (isnumeric(x) && isreal(x) && numel(x)==2 ...
    && all(isfinite(x)) && x(1)<x(2));
end

function restoreLogging(handles,old,model,oldDirty)
for k = 1:numel(handles)
    set_param(handles(k),'DataLogging',old{k});
end
set_param(model,'Dirty',oldDirty);
end



