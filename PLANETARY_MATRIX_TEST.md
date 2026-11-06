# Planetary matrix experiment

Model: `simulation_upgrade.slx`

Subsystem: `DRIVETRAIN/Transmission/Gearbox/AT/Planetary_Matrix_Test`

The new subsystem runs in parallel. The original drivetrain connections remain active.

## Interface

Inputs, matching `Two_Stage_Planetary`:

1. `M_motor`: applied motor torque [N m].
2. `clutch2`: command for the brake on ring 1 (5.419:1 gear).
3. `clutch1`: command for the brake on ring 2 (2.985:1 gear).
4. `w_carrier`: output carrier speed supplied by the existing Shaft [rad/s].

Outputs (the user subsequently reordered the ports; identify them by their names):

- `M_carrier`: torque delivered at carrier 1 [N m].
- `w_engine`: common motor/sun speed [rad/s].
- `constraint_error`: two reconstructed sun speeds minus motor speed [rad/s].

The two MATLAB Function blocks contain the equations. `Matrix_Dynamics` assembles the individual gearset mass matrices; `Shared_Shaft_Speeds` enforces the common suns and the carrier2/ring1 connection. Only motor speed is integrated inside this subsystem. The output acceleration is obtained from the supplied output speed using a Derivative block, as in the previous implementation. Each clutch is a separate copy of the existing LuGre model; its output is subtracted as an opposing applied ring torque. Parameters are read from the existing workspace variables at simulation initialization. Initial motor speed is zero, matching the current standstill scenario.

Additional output inertia is excluded: the existing Shaft/downstream model already accounts for its own inertia. The two gearsets currently use identical pA, pB, pC and tooth counts, matching the model.

## Compare results

Inside AT, open:

- `Matrix_Torque_Comparison`: input 1 is the new block, input 2 is the existing Two_Stage_Planetary.
- `Matrix_Speed_Comparison`: same input ordering.
- `Matrix_Constraint_Error`: both residuals should remain near floating-point roundoff.

The original comparison scopes have subsequently been replaced by the user's scopes `Scope2` (torque), `Scope3` (speed), and `Scope4` (constraints). In Scope2 and Scope3 the input order is matrix model, previous dynamic model, then ratio model. The original verification run remains in the MATLAB workspace as `out_matrix_test`.

This is a parallel evaluation with output speed prescribed by the currently active drivetrain. It does not establish closed-loop vehicle performance when the new torque replaces the active torque. To make that separate experiment, carrier torque must drive Shaft and the matching motor speed must drive the engine-speed feedback. The prescribed-speed derivative interface also needs checking for algebraic-loop behavior when closing that connection. No such rewiring was saved.

## Validation

- AT and the new subsystem passed connectivity and Stateflow lint checks.
- The full model with the parallel block completed 10 s using the current configuration.
- New torque, motor speed, and constraint outputs remained finite.
- Maximum sun-speed constraint residual: approximately 1.09e-11 rad/s.
- Individual gearset and assembled mass matrices were positive definite; assembled eigenvalues approximately 0.0838808 and 0.300262 kg m^2.

These checks verify numerical execution and constraints. They do not validate clutch calibration, shifting performance, or agreement with measured gearbox behavior.

`simulation_upgrade_before_matrix_20261105.slx` preserves the on-disk model from before saving this addition. The working model already had unsaved user edits when this work began; the saved working model includes those edits.

## Comparison with the ratio model: follow-up investigation

The initial implementation assigned clutch commands to the opposite rings from the ratio model. This was corrected in the new block only: `clutch2` now drives Ring1_Clutch, and `clutch1` drives Ring2_Clutch. The user's subsequent scope and output-port edits were preserved.

The existing force calibration provides 750 N m static brake capacity at full command (`N_clutch * n_surfaces * mi_s * r_eq_clutch`). In the ideal steady limit, a grounded ring must react `(ratio - 1) * motor_torque`. At 500 N m motor torque this requires approximately 2210 N m for the 5.419 gear, or 993 N m for the 2.985 gear. The ratio model imposes its ratio without checking this capacity; the LuGre model permits slip.

A temporary SimulationInput override multiplied the `Gain7` force gain inside the two new clutch copies by 6. No force-capacity change was saved. That 10-second run produced:

| At 10 seconds | Matrix with temporary 6x force | Ratio model |
|---|---:|---:|
| Motor speed [rad/s] | 379.759 | 379.761 |
| Output torque [N m] | -1181.99 | -1194.03 |

The reflected motor/gearset inertia in the final gear was approximately 0.848726 kg m^2 at the output. Its predicted deceleration torque contribution was +12.24 N m, compared with an observed +12.04 N m difference. This supports inertia as the source of the remaining final-sample torque difference; slip and shift transients still differ from ideal ratio interpolation.

Workspace results are `out_pg_before` (original command assignment), `out_pg_mapped` (corrected assignment, original capacity), and `out_pg_capacity` (corrected assignment, temporary 6x force). The latter two use the same prescribed output-speed trajectory, with the original drivetrain still active. Do not treat the arbitrary 6x diagnostic factor as a validated physical calibration.

## Oscillation damping follow-up

The user subsequently set each new clutch copy's normal-force gain to 5625 N. Those settings were preserved.

The copied bristle damping term was `sigma1*z_dot`, including when the clutch was disengaged. Since the bristle integrator is held at zero while released but its derivative calculation can remain nonzero, increasing sigma1 alone introduced artificial released-clutch drag. Each new clutch's Subsystem2 now multiplies z_dot by the command clamped to [0,1] before applying bristle damping. The original clutch models were not edited.

Both new copies now use `Matrix_sigma1 = 5000` N s/m, stored in the simulation_upgrade model workspace, rather than the global `sigma1 = 0.289`. The damping contribution is therefore `Matrix_sigma1 * clamp(command,0,1) * z_dot`. Bristle stiffness and viscous friction coefficients were preserved. This is a simulation tuning value, not a measured friction parameter.

Validation used the user's current configuration and 10-second trajectory. Matrix output-torque peak-to-peak variation during 1.0–1.5 s decreased from 285.669 N m to 0.161489 N m. A temporary fixed-step reduction from 1 ms to 0.5 ms gave 0.149827 N m in that interval and the same final torque (-1181.8 N m). The solver step was left at 1 ms. This indicates substantial suppression of clutch ringing in the measured interval; shift and load-change transients can still occur.

The final saved calibration completed the 10-second simulation with finite torque outputs and passed subsystem connectivity/lint checks. Diagnostic results remain in `out_osc_baseline`, `out_osc_damped` (ungated diagnostic, not recommended), `out_osc_gated`, `out_osc_halfstep`, and `out_osc_final`.
