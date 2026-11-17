function [fig, ax, data] = plot_wheel_radii_last_run(varargin)
%PLOT_WHEEL_RADII_LAST_RUN Compare front/rear dynamic radii with r_geom.
%   plot_wheel_radii_last_run
%   [fig,ax,data] = plot_wheel_radii_last_run('XLim',[0 6], ...
%       'RadiusLim',[0.30 0.35]);
%   Options: XLim, RadiusLim (default [] for automatic limits), Rerun
%   (default false), RunID (default latest SDI run).
%   Enables radius logging for future ordinary simulations. If the latest
%   run lacks these signals, captures a fresh simulation automatically.
%   Explicit RunID selects that run only and never silently reruns it.
%   Returned timeseries: r_dynamic_front, r_dynamic_rear, r_geom [m].
%   Geometric radius is captured from the model, not the current workspace.
%   exportgraphics(fig,'wheel_radii.png','Resolution',300)

p = inputParser;
addParameter(p,'XLim',[0 5],@validLimits);
addParameter(p,'RadiusLim',[0.3345 0.3375],@validLimits);
addParameter(p,'Rerun',false,@(x) islogical(x) && isscalar(x));
addParameter(p,'RunID',[],@(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && isfinite(x) && x>0));
parse(p,varargin{:});
opt = p.Results;
model = 'simulation_upgrade';
assert(bdIsLoaded(model),'Open simulation_upgrade before plotting wheel radii.');
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
    'Wait for the simulation to finish before plotting wheel radii.');
names = {'r_dynamic_front','r_dynamic_rear','r_geom'};
sids = [5005 4973 1449];
paths = cell(1,3);
for k = 1:3
    paths{k} = Simulink.ID.getFullName(sprintf('%s:%d',model,sids(k)));
    ph = get_param(paths{k},'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on');
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
            [data,complete] = readRun(run,paths,names);
            opt.Rerun = ~complete;
        end
        if opt.Rerun
            fprintf('Latest run lacks wheel radius signals; capturing a fresh simulation.\n');
        end
    else
        run = Simulink.sdi.getRun(opt.RunID);
        [data,complete] = readRun(run,paths,names);
        assert(complete,'Selected RunID lacks wheel radii. Omit RunID for automatic capture.');
    end
end
if opt.Rerun
    in = Simulink.SimulationInput(model);
    in = in.setModelParameter('SignalLogging','on','SignalLoggingName', ...
        'logsout','DataLoggingOverride',logging);
    out = sim(in);
    ds = out.get('logsout');
    data = struct;
    for k = 1:3
        matches = [];
        for j = 1:ds.numElements
            sig = ds.getElement(j);
            if strcmp(sig.BlockPath.getBlock(1),paths{k}) && sig.PortIndex==1
                matches(end+1) = j; %#ok<AGROW>
            end
        end
        assert(isscalar(matches),'Could not uniquely capture %s.',names{k});
        sig = ds.getElement(matches);
        data.(names{k}) = sig.Values;
    end
end
fig = figure('Color','w','Name','Wheel radii','Position',[100 100 712 658]);
ax = axes(fig);
colors = [0 .36 .65; .85 .33 .10; .25 .25 .25];
styles = {'-','-','--'};
labels = {'Front dynamic radius','Rear dynamic radius','Geometric radius'};
hold(ax,'on');
for k = 1:3
    ts = data.(names{k});
    y = squeeze(ts.Data);
    assert(isvector(y) && numel(y)==numel(ts.Time), ...
        '%s must be a scalar time series.',names{k});
    if k==3 && all(y==y(1))
        yline(ax,y(1),'LineStyle',styles{k},'Color',colors(k,:), ...
            'LineWidth',1.7,'DisplayName',labels{k});
    else
        plot(ax,ts.Time,y,'LineStyle',styles{k},'Color',colors(k,:), ...
            'LineWidth',1.7,'DisplayName',labels{k});
    end
end
grid(ax,'on');
set(ax,'FontName','Arial','FontSize',11,'Box','off','TickDir','out', ...
    'GridAlpha',.15,'XMinorTick','on');
xlabel(ax,'Time (s)');
ylabel(ax,'Wheel radius (m)');
legend(ax,'Location','northeast','Box','on');
if ~isempty(opt.XLim), xlim(ax,opt.XLim); end
if ~isempty(opt.RadiusLim), ylim(ax,opt.RadiusLim); end
end

function [data,complete] = readRun(run,paths,names)
data = struct;
complete = true;
for k = 1:numel(paths)
    matches = [];
    for j = 1:run.SignalCount
        sig = run.getSignalByIndex(j);
        if strcmp(sig.BlockPath,paths{k}) && sig.PortIndex==1 ...
                && sig.NumPoints>0 && isa(sig.Values,'timeseries')
            matches(end+1) = j; %#ok<AGROW>
        end
    end
    if ~isscalar(matches)
        complete = false;
        return
    end
    sig = run.getSignalByIndex(matches);
    data.(names{k}) = sig.Values;
end
end

function tf = validLimits(x)
tf = isempty(x) || (isnumeric(x) && isreal(x) && numel(x)==2 ...
    && all(isfinite(x)) && x(1)<x(2));
end
