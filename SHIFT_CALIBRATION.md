# Automatic transmission shift calibration

Model: `simulation_upgrade.slx`.
Controller: `CONTROLS/Main Control Unit/AT_MCU/AT_MCU`.
Backup: `simulation_upgrade_before_shift_cal_20261108_180630.slx`.

The calibrated values are saved in the model workspace; no initialization script is required.

| Model workspace variable | Saved value | Meaning |
|---|---:|---|
| AT_sync_time | 1.0 s | Motor-speed reference and incoming-capacity ramp duration |
| AT_release_time | 0.4 s | Outgoing-clutch release ramp duration |
| AT_clutch_hold | 0.45 | Incoming-clutch command at completion (normalized 0–1) |
| AT_Kp | 1 | Motor torque proportional correction, N m per rad/s |
| AT_Ki | 2 | Motor torque integral correction, N m per rad |
| AT_torque_rate | 1000 N m/s | Motor command slew limit in synchronization and second gear |
| AT_torque_limit | 500 N m | Symmetric motor command limit |
| AT_slip_start | 10 rad/s | Departure from first-gear speed triggering synchronization, sustained 5 ms |
| AT_speed_tol | 1 rad/s | Final second-gear speed matching tolerance |
| AT_settle_time | 0.03 s | Time the synchronization exit conditions must hold |

The existing `shift_duration = 1` s continues to set the torque-phase polynomial duration. Synchronization now has an independent duration instead of using `shift_duration - t0`.

## Changes

- Bound the torque-phase polynomial to [0,1].
- Capture both clutch commands on entry to synchronization and ramp from those values without an entry discontinuity.
- Use a bounded quintic motor-speed reference toward `i2*w_carrier`.
- Replace the unused PID function with an explicit PI torque correction, conditional integration anti-windup, final command saturation, and torque slew limiting.
- Continue incoming-clutch engagement to its calibrated holding command. Keep that command after synchronization; full pressure is not necessary for holding in this scenario.
- Require reference completion, actual motor-to-second-gear speed matching, and outgoing-clutch release for 30 ms before leaving synchronization.
- Rate-limit return to driver torque in second gear. The same limiter also smooths the later drive-to-braking reversal.

The hard-coded controller time increment is 1 ms, matching the current 1 ms fixed-step model. Changing execution rate requires updating those increments or explicitly preserving a 1 ms controller sample time.

## Measured comparison

Nine candidate calibrations were simulated, followed by a 10 s run and a repeat run using the saved model-workspace values. No acceleration filtering was added.

Jerk below is the first difference of the unfiltered acceleration divided by the sample interval. Both runs use the same existing 1 ms `ode1be` solver. Metrics cover 2.1–5.0 s, encompassing both complete shifts and their settling.

| Measurement | Before | Calibrated |
|---|---:|---:|
| Peak absolute jerk | 1254.9 m/s^3 | 55.08 m/s^3 |
| RMS jerk | 25.68 m/s^3 | 5.25 m/s^3 |
| Minimum acceleration in measurement window | 2.682 m/s^2 | 2.753 m/s^2 |
| Maximum acceleration in measurement window | 6.040 m/s^2 | 6.040 m/s^2 |
| Shift starts | 2.127 s | 2.127 s |
| Synchronization starts | 2.557 s | 2.441 s |
| Shift completes | 2.737 s | 3.471 s |
| Total shift duration | 0.610 s | 1.344 s |
| Maximum absolute motor torque command, full run | 654.94 N m | 500 N m |

Peak jerk decreased 95.6%; RMS jerk decreased 79.5%. The cost is a longer shift. Some acceleration humps and dips remain; this is an improved candidate calibration, not an optimum or disturbance-free shift.

The full calibrated run produced finite outputs and completed synchronization. The final speed error at 10 s was approximately -0.000121 rad/s. There is a small transient speed deviation after leaving synchronization as motor torque returns to driver demand; matching at the exit does not imply zero error for every later sample.

The 7.9–9 s braking-window minimum acceleration changed from -4.176 to -3.214 m/s^2. This comes with the response delay of the motor torque slew limit. Startup acceleration is unchanged, with a maximum of 6.095 m/s^2 in the first second.

Controller connectivity and Stateflow lint checks passed. An existing unused `cr2` input is preserved to keep the chart interface compatible; MATLAB emits an unused-input warning during compilation.

## Review and reproduce

- `images/shift_calibration_comparison.png`: before/after acceleration around the shift.
- `shift_calibration_results.mat`: saved parameter names/values, baseline metrics, verified output, and backup path.
- `result = run_shift_calibration([],10)`: rerun the saved calibration with temporary diagnostic logging; the helper restores the previous logging selection.
- `measure_shift_calibration.m`: measurement definitions.

Validation covers the current forward powered 1-to-2 scenario. Downshifts, other loads, low-friction operation, clutch heat/slip energy, and solver-step convergence were not validated. The new clutch-holding command and gains should be reevaluated for those conditions. Planetary and clutch physical parameters were not recalibrated.
