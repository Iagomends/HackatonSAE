# MATLAB / Simulink reorganization audit

## Inspection before edits

Model: `EV_Backward_Baseline.slx`, MATLAB/Simulink R2024b Update 1.
The loaded model had unsaved edits when work started. Those edits were retained;
the regression reference and block/layout snapshot use that loaded model.

The model workspace was empty. Initialization/start/stop/load callbacks were
empty. The solver was `FixedStepDiscrete`, with step `EV_dt`, start `0.0`,
stop `EV_stop`, and signal logging to `out.logsout` enabled.

`init_ev_backward.m` defined 53 variables, interleaving these groups:

| Group | Existing parameters/data |
|---|---|
| Simulation and cycle | `EV_dt`, `EV_cycle_time`, `EV_cycle_speed`, `EV_time`, `EV_speed`, `EV_drive_cycle`, `EV_v0`, `EV_stop` |
| Vehicle | `EV_mass`, `EV_g`, `EV_rho`, `EV_Cd`, `EV_area`, `EV_Crr`, `EV_grade` |
| Wheels/transmission | `EV_radius`, `EV_gear`, `EV_final`, `EV_drive_eta` |
| Topology | `EV_topology` (1 = fixed gear, 2 = planetary) |
| Motors | `EV_motor_eta`, `EV_regen_eta`, vehicle power/torque ratings, Motor 1/2 size/speed/regen limits, single-motor and e-CVT derived ratings, active-rating compatibility aliases |
| Planetary | `N_s`, `N_r`, `ecvt_final_ratio`, `ecvt_final_efficiency`, `ecvt_parameters` |
| Battery | `EV_voltage`, `EV_resistance`, `EV_capacity_Ah`, `EV_SOC0` |
| Aging | `degradation_Crate`, `degradation_scale`, `degradation_rate`, `EV_SOH0` |

The physical block parameters already referenced these workspace variables.
Configurable literals remained in the SOC assertion, SoH saturation, initial
Ah throughput, and simulation start time. Mathematical constants, conversion
factors, signal indices, zero-crossing switch thresholds, and solver algorithm
constants were distinguished from configurable physical parameters.

`plot_ev_ecvt.m` contained eight panels: both motor speeds, torques, mechanical
powers, electrical powers, battery power/current, SOC and SoH.
`plot_ev_soh.m` contained six: SOC, SoH, current, C-rate, degradation coefficient
and actual capacity-loss rate. Both exported figures automatically. Vehicle,
wheel, ring/objective/feasibility and accumulated-degradation plots were missing.

There were 20 common logged signals and 18 additional e-CVT signals. Existing
logging was sufficient for every requested plot; no logging or wiring changes
were necessary.

## Final organization

- `initialize_model.m`: all parameters/input data, grouped by subsystem with
  units and user-defined/derived comments. `EV_topology` is defined only in
  **Powertrain Variant Selection**. Both sets of motor ratings are always
  derived, preserving `SimulationInput` variant overrides.
- `plot_results.m`: all figures, formatting helpers, optional comparison of
  completed runs, optional PNG/FIG export. No simulation, initialization or
  automatic loading of potentially stale result files.
- Old initialization/plot compatibility entry points: removed in the subsequent
  cleanup; use the canonical script names above.
- Run/check scripts: use the canonical entry points; physical assertions remain
  unchanged. The e-CVT calculation functions retain their algorithms.

The only changed block/model parameter expressions are:

| Block SID / model setting | Before | After |
|---|---|---|
| 118 `Expr` | `(u >= 0) && (u <= 1)` | `(u >= EV_SOC_min) && (u <= EV_SOC_max)` |
| 122 `InitialCondition` | `0` | `EV_Ah0` |
| 155 `LowerLimit` | `0` | `EV_SOH_min` |
| 155 `UpperLimit` | `1` | `EV_SOH_max` |
| Model `StartTime` | `0.0` | `EV_start` |

The new variables retain the original numeric values. The degradation curve's
relative shape is now named `degradation_relative_loss`; the resulting
`degradation_rate` is identical. The four existing external harness registrations
were preserved, with their metadata associated with the saved model identity.

Constants such as `1/3600` (A to Ah/s), `0.5` (two half-cycles per full cycle),
unity in `SoH = 1-D`, and zero power sign thresholds remain in the equations.
The variant conditions `EV_topology == 1` and `EV_topology == 2` are unchanged.

## Regression results

`reorganization_baseline.mat` contains both complete pre-edit SimulationOutputs,
the original 53 parameter values, and the block/dialog/layout snapshot.
`verify_model_reorganization.m` reruns both default variants and compares every
logged signal's time/data arrays using `isequaln`, plus the full simulation time
grid, block inventory, positions, and all unaffected dialog parameters.

| Topology | Signals | Samples per signal | Maximum absolute difference |
|---|---:|---:|---:|
| Fixed gear | 20 | 1601 | 0 |
| Planetary e-CVT | 38 | 1601 | 0 |

All 53 original initialization variables are identical. Both runs end at 160 s
with SOC `0.794947467089583` and SoH `0.999998731850121`.
The model connectivity and Stateflow lint checks report healthy status.

Additional checks passed:

- `run_ev_backward`: finite signals, motoring/regeneration, battery power
  balance, SOC/Ah/EFC integration, monotonic degradation, lookup interpolation
  and zero-current aging behavior.
- `run_ev_ecvt`: planetary kinematics/torques/power balance, motor envelopes,
  electrical efficiencies, optimal operating points and battery/SoH integration.
- `check_ecvt_limits`: nine extreme requests and independent objective sweeps.
- Plotting: five figures for fixed gear, six for e-CVT, and seven with an
  explicit two-run comparison. e-CVT and degradation exports inspected visually.
- The exact unassigned `initialize_model; sim(...); plot_results` workflow,
  plotting after clearing initialization variables, and optional PNG/FIG export.
- MATLAB Code Analyzer: zero findings in the new initialization, plotting and
  regression scripts.

After verification, unused legacy scripts, root-level generated results/figures,
the old architecture image, and simulation caches were removed during cleanup.
The runners regenerate result files; the regression reference and external
test harnesses were retained. Pre-existing working-tree deletions were untouched.

These comparisons cover the supplied default cycle and both existing variants;
they are not a claim of validation for every possible future parameter set.
