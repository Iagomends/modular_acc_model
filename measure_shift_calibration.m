function result = measure_shift_calibration(out)
% Measure actual speed synchronization and acceleration disturbance.
ds = out.logsout;
for k = 1:ds.numElements
    sig = ds{k};
    path = sig.BlockPath.getBlock(1);
    if endsWith(path,'/AT_MCU/AT_MCU')
        switch sig.PortIndex
            case 1, motorTorque = sig.Values;
            case 2, phase = sig.Values;
            case 7, onCommand = sig.Values;
        end
    elseif endsWith(path,'/Motor_Speed')
        motorSpeed = sig.Values;
    elseif endsWith(path,'/AT/Shaft') && sig.PortIndex == 2
        carrierSpeed = sig.Values;
    elseif strcmp(sig.Name,'acc')
        acceleration = sig.Values;
    end
end
t = phase.Time;
p = squeeze(phase.Data);
start = find(p == 1,1);
sync = find(p == 2,1);
finish = [];
if ~isempty(sync), finish = find((1:numel(p))' > sync & p == 0,1); end
result.completed = ~isempty(finish);
result.shiftStart = NaN;
result.syncStart = NaN;
result.shiftEnd = NaN;
if ~isempty(start), result.shiftStart = t(start); end
if ~isempty(sync), result.syncStart = t(sync); end
if ~isempty(finish), result.shiftEnd = t(finish); end
window = t >= 2.1 & t <= min(5,t(end));
a = squeeze(acceleration.Data);
j = diff(a)./diff(acceleration.Time);
jmask = acceleration.Time(2:end) >= 2.1 & acceleration.Time(2:end) <= min(5,t(end));
result.accMin = min(a(window));
result.accMax = max(a(window));
result.peakJerk = max(abs(j(jmask)));
result.rmsJerk = sqrt(mean(j(jmask).^2));
result.peakTorque = max(abs(motorTorque.Data));
ratio = evalin('base','i.min');
result.finalSpeedError = motorSpeed.Data(end)-ratio*carrierSpeed.Data(end);
result.finalOnCommand = onCommand.Data(end);
result.finite = all(isfinite(a)) && all(isfinite(motorSpeed.Data));
end
