# Pure-electric planetary e-CVT

Run `run_ev_ecvt` from this folder to initialize, select topology 2, simulate
the existing 160 s cycle, validate every sample, and produce
`EV_eCVT_results.mat`, `EV_eCVT_plots.png` and `EV_eCVT_plots.fig`.
For interactive runs, run `init_ev_backward`, set `EV_topology = 2`, and Run.
Set `EV_topology = 1` to return to the existing fixed-gear choice.

The new choice is `Transmission/Planetary_eCVT_Two_Motors`. The original
variant was not changed. The vehicle, drive cycle, battery and degradation
equations were not changed. All variant ports retain their names, order,
scalar dimensions and SI units.

## Mechanical equations and sign convention

Motor 1 is attached to the sun, Motor 2 to the carrier, and the ring to the
final drive. The planetary is ideal, with no inertias or friction. The final
drive includes the lumped `ecvt_final_efficiency` loss.

Let `a = N_s/N_r`, `b = 1+a`. Motor torques are applied TO the gearset. `T_R`
is delivered BY the ring to the output, so the external torque applied to the
ring gear is `-T_R`.

```text
a*w_s + w_r = b*w_c                  Willis, valid also at standstill
T_M1 = -a*T_R
T_M2 =  b*T_R
T_M1 + T_M2 - T_R = 0                torque equilibrium
T_M1*w_s + T_M2*w_c = T_R*w_r         ideal mechanical power balance
```

The torque relations follow from virtual work on the speed constraint:
the applied torque vector is proportional to `[a, 1, -b]` at the
sun/ring/carrier ports. Thus the two torques cannot be independently split.
The available control freedom is **internal speed selection**.

The ring request is `w_r = final_ratio*w_wheel`. For nonnegative requested
wheel power, `T_R_req = T_wheel/(final_ratio*eta_final)`; for regeneration,
`T_R_req = T_wheel*eta_final/final_ratio`. Negative travel is supported using
power signs, not speed signs alone.

## Exact speed selection, without Optimization Toolbox

The shared function `ecvt_operating_point.m` minimizes

```text
J = (w_s/w_M1_max)^2 + (w_c/w_M2_max)^2
w_s = (b*w_c-w_r)/a
```

The unconstrained minimizer is

```text
w_c_free = b*w_r*w_M2_max^2 / (b^2*w_M2_max^2 + a^2*w_M1_max^2)
```

For fixed requested torque, both motor torques are fixed. Each motor has
`abs(T)<=Tmax`, `abs(w)<=wmax`, `abs(T*w)<=Pmax`, equivalently a
constant-torque/constant-power envelope. Generating torque/power bounds are
multiplied by that motor's regeneration fraction. All four shaft-speed sign
combinations are considered; in each, power signs and envelope bounds are
known. These constraints produce a closed interval of feasible carrier
speeds. Project `w_c_free` onto each nonempty interval and choose the lowest
J. This is the global scalar quadratic minimum, including generating cases.

If torque demand is infeasible, 55 bisection steps find the largest feasible
fraction of the requested ring torque; the planetary torque ratios are
preserved, and speed minimization is then repeated. No torque is silently
redistributed between machines.

The maximum kinematically achievable absolute ring speed is
`a*w_M1_max+b*w_M2_max`. Beyond this, no solution can both preserve imposed
ring speed and respect motor speed bounds. The model caps the achievable
ring speed, logs a speed shortfall, and marks the request infeasible.
The prescribed vehicle cycle is not altered. Any flagged sample therefore
represents an **unserved operating request**, not successful tracking of that
cycle. Battery energy at such samples covers delivered power only.

## Preserving the existing scalar interface

The variant outputs remain `motor_mechanical_power`, `motor_torque` and
`motor_speed`. For the e-CVT these represent the **equivalent ring shaft**:
`T_R*w_r`, `T_R`, `w_r`, not an individual electric motor. The true two-machine
quantities are logged explicitly with `motor1_...` and `motor2_...` names.

The existing external Motor subsystem now has a topology-selected electrical
branch. Its original efficiency blocks and single-motor computation are intact.
For topology 2, `ecvt_electrical_power.m` uses the same shared operating-point
function to reconstruct the two motors from delivered ring torque/speed.
The unique minimum makes reconstruction deterministic, including saturated
torque cases. This avoids adding ports, hidden global signal routing, or
misrepresenting electrical power as mechanical power.

Each motor uses exactly the original efficiency law:

```text
P_el_i = P_mech_i / EV_motor_eta      if P_mech_i >= 0
P_el_i = P_mech_i * EV_regen_eta      otherwise
P_battery = P_el_1 + P_el_2
```

The sum reaches the existing battery once; efficiencies are not applied twice.
Its signed current feeds the existing SOC and C-rate-dependent SoH paths.
This baseline uses Interpreted MATLAB Function blocks to call the shared,
readable `.m` functions, so keep those files on the MATLAB path. It is a
normal-mode simulation prototype, not a code-generation implementation.

## Parameters in init_ev_backward.m

| Parameter | Default | Meaning |
|---|---:|---|
| `EV_topology` | 1 | Existing topology 1; new e-CVT 2 |
| `vehicle_max_motor_power` | 160000 W | Total installed mechanical machine power |
| `vehicle_max_motor_torque` | 900 Nm | Sum of machine torque ratings, not wheel torque |
| `motor1_size_fraction`, `motor2_size_fraction` | 0.5, 0.5 | Positive fractions; sum must equal 1 |
| `motor1_wmax`, `motor2_wmax` | 1256.637 rad/s | Shaft speed limits (12000 rpm) |
| `motor1_regen_fraction`, `motor2_regen_fraction` | 1, 1 | Generating torque and power rating fractions |
| `N_s`, `N_r` | 30, 78 | Sun/ring tooth counts; planet has 24 teeth |
| `ecvt_final_ratio` | `EV_final` = 3 | Ring/wheel speed ratio |
| `ecvt_final_efficiency` | `EV_drive_eta` = 0.96 | Lumped downstream efficiency |
| `EV_motor_eta`, `EV_regen_eta` | 0.92, 0.85 | Shared motor/inverter efficiencies |

Single-motor ratings equal the vehicle ratings. The two e-CVT ratings are
fraction times vehicle ratings: defaults are 80 kW and 450 Nm per machine.
Both sets (`single_motor1_*`, `ecvt_motor1_*`, `ecvt_motor2_*`) are derived
centrally. `motor1_Pmax/Tmax` and `motor2_Pmax/Tmax` are convenience aliases
for the topology selected when initialization runs. The e-CVT always uses
its derived `ecvt_parameters`, allowing SimulationInput topology overrides
without stale sizing. Edit the topology line before initialization if you
also need the convenience aliases to reflect topology 2.

The historical single-motor model had no envelope enforcement. It remains
unchanged as requested; the new sizing variables do not retroactively limit
its existing behavior. Re-run initialization after editing ratings/fractions.

## Logs and validation

In addition to all previous logs, the new choice logs:

- `ecvt_requested_ring_torque`, `ecvt_requested_ring_speed`
- `ecvt_ring_torque`, `ecvt_ring_speed`, `ecvt_mechanical_power`
- `motor1_sun_speed`, `motor2_carrier_speed`
- `motor1_torque`, `motor2_torque`
- `motor1_mechanical_power`, `motor2_mechanical_power`
- `motor1_electrical_power`, `motor2_electrical_power`
- `ecvt_total_electrical_power`, `ecvt_objective_J`
- `ecvt_feasible` (1=request achieved, 0=infeasible)
- `ecvt_torque_shortfall`, `ecvt_speed_shortfall`

`wheel_torque` remains the logged requested wheel/output torque. `battery_power`,
`battery_current`, `SOC` and `SOH` retain their original names.

`run_ev_ecvt` verifies Willis, torque equilibrium, ideal planetary and lossy
final-drive power balance, all motor envelopes, electrical sign/efficiency,
the electrical adapter, J and the unconstrained optimum at every default-cycle
sample, plus battery current, SOC, Ah and SoH integration. `check_ecvt_limits`
exercises nine extreme/constrained requests and compares against independent
dense feasible-speed sweeps. `ecvt_planetary.feature` contains component-level
Simulink scenarios for motoring, regeneration, torque clipping and overspeed.

The default cycle is 100% feasible across 1601 samples. Final SOC is
0.79494747, final SoH 0.9999987319, and throughput 1.580464 Ah. All 20 existing
signals are bit-for-bit unchanged with topology 1 selected. Equal energy usage
between topologies on this cycle is expected: constant efficiencies, equal
lumped drivetrain loss and no opposite-direction power circulation imply the
same battery demand. Speed minimization does not itself save modeled energy
without speed-dependent losses or an efficiency map.

All four Simulink component scenarios passed in draft and full-compilation
modes (12/12 assessments). Select `EV_topology=2` in the workspace before
running full-mode component tests, because Simulink Test cannot create a
harness for an inactive variant choice. Structural connectivity checks passed.
