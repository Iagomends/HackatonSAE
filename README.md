# HackatonSAE

## MATLAB / Simulink workflow

From this folder:

```matlab
initialize_model
out = sim("EV_Backward_Baseline");
plot_results
```

`initialize_model.m` is the single parameter source. Its `%%` sections group
parameters by subsystem, distinguish user inputs from derived values, and
document units. Change `EV_topology` in **Powertrain Variant Selection**:
`1` is the original single motor/fixed gear; `2` is the two-motor planetary e-CVT.
Initialization defines parameters/input data and validates them; it does not
configure blocks, run a simulation, or create figures. The model does not
automatically rerun initialization, so subsequent workspace edits are preserved.

`plot_results.m` reads `out.logsout` and contains all visualization code.
It also accepts the `ans` SimulationOutput from an unassigned `sim` call when
`out` is absent; explicit assignment avoids selecting an older `out`.
Its figures cover vehicle dynamics, wheels, motor(s), planetary operation,
battery, and degradation. Motor 2 and planetary plots appear only for e-CVT
logs. Plotting saved results does not require running initialization again.

To compare completed runs without changing the default selector:

```matlab
initialize_model
in = Simulink.SimulationInput("EV_Backward_Baseline");
out_fixed = sim(in.setVariable('EV_topology',1));
out_ecvt = sim(in.setVariable('EV_topology',2));
out = out_ecvt;
comparison_outputs = {out_fixed,out_ecvt};
comparison_labels = {'Fixed gear','Planetary e-CVT'};
plot_results
```

Set `plot_export_folder = 'results'` before plotting to export each figure as
PNG and FIG. Otherwise no figures are written to disk. Clear
`comparison_outputs` to stop showing the optional comparison.

Use `initialize_model` and `plot_results`; the obsolete compatibility scripts
have been removed. `run_ev_backward` and `run_ev_ecvt` remain optional
simulate/check/save/plot workflows. Model calculation functions stay in
`ecvt_operating_point.m` and `ecvt_electrical_power.m`.

Simulation caches, previous result MAT files, and legacy figure exports are
not required to run the model and have been removed. The run scripts regenerate
their result files on demand. The external test harnesses in `slprj/mbd_agent`
and the regression reference in `validation` remain part of the verification setup.

Run `verify_model_reorganization` to compare both default topologies with
the captured pre-reorganization traces in `validation/reorganization_baseline.mat`.
See [reorganization audit](validation/reorganization.md),
[vehicle equations](README_EV_Backward.md), and [e-CVT equations](README_eCVT.md).
