# Route-aware powertrain comparison

Run `run_route_comparison`. Initialization, the existing
`EV_Backward_Baseline.slx` model, and `plot_results.m` remain the three main
parts of the workflow. Helper functions implement the control calculations.
The model's Vehicle, Battery, and Battery Degradation subsystems are unchanged.

The runner uses the same 160-second, 1.9625 km predefined route, 1,800 kg
vehicle, 360 V / 0.08 ohm / 150 Ah battery, 80% initial SOC, and installed
160 kW / 900 Nm for every case. The single motor receives the total rating;
the two e-CVT motors receive the existing fractions of that rating. Motor
efficiency laws, SOC integration, aging curve and all numerical vehicle
parameters remain unchanged. No case-specific calibration is applied.

## Cases

1. **Direct fixed gear:** the existing single-motor gear/final-drive ratios
   and efficiencies, conventional maximum available regeneration. The route
   branch explicitly checks the installed speed/torque/power limits and
   assigns unavailable motor braking to friction brakes.
2. **Local degradation e-CVT:** the existing `ecvt_operating_point` controller,
   minimizing instantaneous `k(C)*abs(I)/3600` only. It receives the current
   torque/speed request and plant parameters, with no route, SOC reserve or
   terminal target. Its existing full motor-braking request is retained;
   internal circulation can dissipate energy in motor losses. Friction covers
   any braking demand beyond its feasible motor envelope. It is not forced
   above 20% SOC.
3. **Predictive e-CVT:** the same planetary, motor envelopes and battery,
   with full-route degradation planning, feedback from actual SOC and the
   current route index, and controllable motor/friction brake allocation.
   `SOC_final >= SOC_terminal_min` is enforced whenever reachable.

The preferred parameters in `initialize_model.m` are:

```matlab
SOC_min_preferred = 0.20;
SOC_max_preferred = 0.80;
SOC_terminal_min  = 0.20;
```

The preferred range is shown in plots and used to break exact predictive
cost ties. It is not a hard SOC path constraint. The existing physical
0–100% battery limits remain; full-charge protection reduces regeneration
and assigns the difference to friction. The terminal constraint applies only
to Case 3. No charging after the route is simulated or counted in route loss.
The assumption of later slow charging is represented by having no reward for
surplus terminal SOC.

## Remaining-route requirement and terminal guarantee

`prepare_route_control` evaluates the complete known speed profile using the
same discrete acceleration and vehicle force equations as the plant. At each
sample it finds:

- Minimum feasible traction current `Itraction` (minimizing current also
  minimizes traction degradation under the existing monotone aging model).
- Maximum feasible charging magnitude `IchargeMax`, allowing brake fraction
  and both motor speeds to vary under all planetary/motor constraints.

For braking, variables `t=abs(Tr)` and `z=Pm2` make the constraints linear:
`Pm1=-abs(wr)*t-z`. In each motor power-sign region all torque, speed, power
and brake-fraction bounds form a bounded 2-D polytope. `ecvt_max_regen`
enumerates its vertices and chooses the most negative electrical power.
Scaling that point's motor torques and powers preserves its speeds and
feasibility, so every charging current from zero to its maximum is achievable.

Starting with `R(end)=SOC_terminal_min`, the exact backward reachability
recursion is:

```text
R(k) = max(0, R(k+1) + dt*(Itraction(k)-IchargeMax(k))/(3600*Qnom))
SOC_required_to_finish(k) = R(k)
```

This uses battery current from `Pbat=(Voc-R*I)*I`, including resistance, not
simply wheel energy divided by nominal battery energy. A required SOC above
100%, an infeasible traction/speed request, or current SOC below this bound
is explicitly reported as infeasible. The zero floor in the backward
recursion protects physical SOC, not the preferred 20% level.

`SOC_needed_without_regen` uses the same recursion with zero charging. Above
that bound, zero regeneration is optimal: charging would add degradation
without helping any constraint. Distance and progress use the same left-step
time discretization as the battery integrators:

```text
distance(k+1) = distance(k) + abs(speed(k))*dt
remaining_distance = distance(end)-distance(k)
route_progress = distance(k)/distance(end)
```

## Predictive degradation optimization

The controller builds a backward Bellman value table over SOC and route index:

```text
V(k,s) = min_q { dt*kdeg(q/Qnom)*q/3600 + V(k+1,s+dt*q/(3600*Qnom)) }
0 <= q <= IchargeMax(k)                         during braking
V(end,s) = 0 for s >= SOC_terminal_min          terminal condition
```

Traction has the fixed minimum-current transition and its aging cost. The
existing lookup is not assumed convex. The value table and charging action
grid therefore handle its slope changes without fitting a different curve.
Default numerical resolution is 301 SOC nodes and 41 charging-current nodes,
plus all aging breakpoints and exact feasibility-bound actions. Value
interpolation makes the cost optimum a numerical approximation; it is not
claimed to be the exact continuous global optimum. Doubling both resolutions
changed degradation by 0.001254% in a partial-regeneration validation case.

At every simulation sample, `route_regen_policy` queries that table using
**live SOC**, not a pre-recorded open-loop regen schedule. It includes the
exact minimum charging action needed to keep the next SOC at or above
`R(k+1)`, independent of the grid. Thus terminal feasibility does not depend
on the numerical cost resolution. This guarantee assumes the prescribed
route and existing deterministic plant model are followed. A changed speed
profile must be prepared again; a mismatch raises an explicit error.

The selected charging current is converted to exact battery power and a
feasible scaled motor operating point. Early gentle regeneration can be
preferred over later high-current charging when reserve is needed. If the
terminal target is already unreachable, the flag stays false and the
controller maximizes useful available recovery; it does not silently assert
success or fabricate SOC.

## Braking energy and model integration

`regen_fraction` is the fraction of requested wheel braking assigned to the
motors. It is zero outside braking and lies in [0,1]. Because the local
controller can circulate power, a fraction of 1 does not imply all braking
energy reaches the battery. Positive friction/loss logs make that explicit:

```text
Pbat = Pwheel + Pfriction + Pdrivetrain_loss + Pmotor_loss
Voc*I = Pbat + R*I^2
```

The dashboard's braking-energy bar partitions wheel braking energy into
battery-terminal recovery, friction, drivetrain losses and motor losses.
Battery ohmic loss is logged separately; adding it to that bar would double
count part of battery-terminal recovery.

`Route_Powertrain` is a new controller/dispatch branch in the existing model.
`EV_route_case=0` preserves the original topology selection. Cases 1–3 select
the route branch's electrical power as the **only** battery input. Existing
legacy transmission/motor diagnostic branches remain for compatibility but
do not drive the battery in route mode. Use `route_motor*` logs for the actual
route-mode operating points. The original physical battery and aging
subsystems integrate the dispatched power; SOC/SOH are not computed only in
postprocessing. All 17 original logged signals are bit-for-bit unchanged
with route mode disabled.

## Outputs and interactive use

`run_route_comparison` writes `EV_route_comparison.mat`,
`EV_route_case_1.csv` through `_3.csv`, and `EV_route_summary.csv`. It calls
`plot_results` to create a 12-panel dashboard and a separate summary-table
window. MAT files retain the route plan, configuration, full SimulationOutput
objects and per-case tables. All power/current/energy signs and units are
explicit in the plots and table headers.

For a single interactive predictive run:

```matlab
initialize_model
EV_route_plan = prepare_route_control(EV_route_config);
EV_route_case = 3;
EV_topology = 2;
out = sim('EV_Backward_Baseline');
```

Edit physical/route parameters in `initialize_model.m` and reinitialize/rebuild
the plan. The controller configuration is a snapshot of those parameters;
if overriding plant parameters programmatically, pass a correspondingly
updated `EV_route_config` and rebuild `EV_route_plan` in SimulationInput.
Initial-SOC-only overrides require no change to the Bellman table because
the feedback uses the actual battery SOC.

For the comparison dashboard, use `run_route_comparison` or load the saved
MAT file and call `plot_results`. Set `plot_export_folder='results'` to export
PNG/FIG files. Clear `route_comparison` to return to legacy single-run plots.

## Initial-SOC sensitivity route

The prescribed route is now the named `motorway_service_area` profile: an
early slowdown, a later sustained 33 m/s high-energy section lasting about
21 minutes, and a final stop. The route is approximately 47 km. The longer
cruise is intentionally sized so backward reachability places the 30% and
35% cases near or below the terminal reserve while 40–50% have more room to
avoid unnecessary charging. In the current model, the reserve is 34.5832%
with maximum useful regeneration and 35.1404% with none. `run_route_sensitivity` saves the
study table and plots predictive current SOC, `SOC_required_to_finish`,
regeneration fraction, route progress, wheel power, and remaining energy.

The compact main dashboard now has five comparison panels: route speed and
progress; SOC for all three cases plus predictive reserve and 20/80% lines;
battery C-rate; accumulated capacity loss; and returned versus friction-brake
energy. Internal battery power, current, wheel power, motor points, and loss
signals remain logged and available in the CSV/MAT outputs but are not shown
in the presentation dashboard.

The summary includes final/minimum SOC, route SOH loss in ppm, regenerated
and friction energy, peak C-rate, time below 20%, and loss in ppm/100 km.
Energy and below-threshold time use left rectangles consistent with the
plant's Euler integration; the final sample does not add an extra interval.

On the unchanged short route at 80% initial SOC, no regeneration is necessary
for Case 3. Its lower aging is therefore mainly the effect of allowing
friction braking instead of charging/circulation. These default results do
not by themselves establish a benefit of preview over a local controller
with the same free friction-braking decision. The separate low-SOC tests
exercise the predictive terminal constraint without retuning the comparison.
