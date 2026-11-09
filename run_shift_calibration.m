function result = run_shift_calibration(values, stopTime)
% Diagnostic calibration run; overrides model workspace values for one run.
names = {'AT_sync_time','AT_release_time','AT_clutch_hold','AT_Kp', ...
    'AT_Ki','AT_torque_rate','AT_torque_limit','AT_slip_start', ...
    'AT_speed_tol','AT_settle_time'};
if nargin < 2, stopTime = 10; end
if nargin < 1 || isempty(values)
    mw = get_param('simulation_upgrade','ModelWorkspace');
    values = cellfun(@(name) mw.getVariable(name),names);
end
% Restore the user's logging selection even if simulation fails.
sids = [874 874 874 874 8564 755 8745 8803 213];
ports = [1 2 6 7 1 2 1 1 1];
paths = cell(size(sids));
wasLogged = false(size(sids));
for k = 1:numel(sids)
    paths{k} = Simulink.ID.getFullName(sprintf('simulation_upgrade:%d',sids(k)));
    handles = get_param(paths{k},'PortHandles');
    wasLogged(k) = strcmp(get_param(handles.Outport(ports(k)),'DataLogging'),'on');
end
cleanup = onCleanup(@() restoreLogging(paths,ports,wasLogged)); %#ok<NASGU>
for k = 1:numel(paths)
    Simulink.sdi.markSignalForStreaming(paths{k},ports(k),'on');
end
in = Simulink.SimulationInput('simulation_upgrade');
in = in.setModelParameter('StopTime',num2str(stopTime),'SignalLogging','on');
for k = 1:numel(names)
    in = in.setVariable(names{k},values(k),'Workspace','simulation_upgrade');
end
out = sim(in);
result = measure_shift_calibration(out);
result.values = values;
result.out = out;
end

function restoreLogging(paths,ports,wasLogged)
for k = 1:numel(paths)
    if ~wasLogged(k)
        Simulink.sdi.markSignalForStreaming(paths{k},ports(k),'off');
    end
end
end
