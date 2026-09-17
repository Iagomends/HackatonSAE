# Backward-facing EV baseline

An additional pure-electric planetary e-CVT is available as `EV_topology = 2`.
See [README_eCVT.md](README_eCVT.md) for equations, sizing, interface semantics,
limits, validation and plots. Run `run_ev_ecvt` to exercise it. Topology 1
retains the original behavior described below.

Run `initialize_model`, then open `EV_Backward_Baseline.slx` and press Run.
Run the initialization script once before compiling or simulating; it is not
automatically rerun, so workspace parameter changes are preserved.
`run_ev_backward.m` initializes, simulates, checks the results and saves them.

The prescribed speed is the independent input. There is no driver or forward
vehicle dynamics. Replace the speed/time knots in `initialize_model.m` to use
another cycle. Speed is in m/s, time in seconds and grade in radians. The
sampled speed is differentiated by a backward difference at `EV_dt` seconds;
initial acceleration is zero. Discontinuous speed inputs should be avoided.

## Subsystems and sign conventions

- **Drive_Cycle:** workspace speed source and finite difference acceleration.
- **Vehicle:** `F = m*a + 0.5*rho*Cd*A*v*abs(v) +
  Crr*m*g*cos(grade)*sign(v) + m*g*sin(grade)`;
  `T_w = F*r`, `omega_w = v/r`, `P_w = T_w*omega_w`.
  Rolling resistance is zero at standstill. Forces and torques are signed.
- **Transmission:** Variant Subsystem, selected by `EV_topology == 1`,
  containing a single motor with fixed gear and differential. Total
  ratio is `EV_gear*EV_final`; `omega_m = ratio*omega_w`.
  When wheel power is nonnegative, `T_m = T_w/(ratio*eta_d)`;
  during regeneration, `T_m = T_w*eta_d/ratio`.
- **Motor:** `P_m = T_m*omega_m`. Electrical power is `P_m/eta_m` in
  motoring and `P_m*eta_regen` during regeneration. Efficiencies include
  inverter losses.
- **Battery:** constant open-circuit voltage and series resistance:
  `P_bat = (V_oc - R*I)*I`. The low-current physical root is evaluated as
  `I = 2*P_bat/(V_oc + sqrt(V_oc^2 - 4*R*P_bat))`, which also works at
  zero resistance. Positive current discharges the pack.
  `dSOC/dt = -I/(3600*capacity_Ah)`. Battery power is terminal power;
  chemical power is `V_oc*I` and resistance loss is `R*I^2`.
- **Battery Degradation:** absolute current integration in Ah, then
  `EFC = Ah_throughput/(2*capacity_Ah)`. A full discharge and full recharge
  count as one equivalent full cycle. Integrators use forward Euler at
  `EV_dt`; throughput begins at zero independently of initial SOC.
- **Outputs:** named signals are logged in `out.logsout` as a Dataset.

All physical assumptions are in `initialize_model.m`. Battery power above
`V_oc^2/(4*R)` (for R > 0) or SOC outside [0,1] terminates simulation through
assertions. This is an unconstrained demand calculation: motor torque/speed,
battery charge-current limits, friction brake blending, auxiliary loads,
temperature, rotating inertia and capacity fade are not modeled. Negative
wheel power is assumed fully available to the regenerative path before losses.

The throughput metric measures usage. The separate C-rate-dependent SoH
estimate described below converts that usage into illustrative capacity loss.
It has no feedback into SOC, nominal capacity, voltage or resistance.

For comparisons, keep the transmission interface (wheel torque/speed to motor
torque/speed/mechanical power) consistent. Multiple independently controlled
motors will also require extending the motor interface or using a bus.
To add a topology, add another variant choice inside Transmission with matching
port names and a distinct condition such as `EV_topology == 2`.

## Baseline validation

Compiled and simulated in MATLAB/Simulink R2024b. The 160 s demonstration
produced all 15 logged signals, with final SOC 0.794947, throughput 1.580464 Ah
and 0.00526821 EFC. Current ranged from -101.006 A to 220.839 A.
Structural checks found no disconnected ports or dangling lines. The run
script checked finite outputs, motoring and regeneration, battery terminal
power balance, nonnegative drivetrain/motor losses, monotonic throughput,
SOC integration and EFC scaling. Results are saved in
`EV_Backward_results.mat` as the SimulationOutput variable `out`.

## C-rate-dependent SoH extension

Only the degradation calculations and their input/output wiring were extended.
The battery's electrical and SOC equations, drive cycle, vehicle, transmission
and motor are unchanged. The subsystem is named **Battery Degradation**.
Its inputs are **Battery Current** (signed A) and **Battery Capacity** (nominal
Ah, held constant during a run). Its outputs are **Ah Throughput**, existing
**equivalent_full_cycles**, **C-rate**, **Degradation Rate**,
**Accumulated Degradation** and **SoH**.

At sample k, with dt = EV_dt:

```text
C[k]       = abs(I[k])/Qnom
kdeg[k]    = linear_lookup(C[k], degradation_Crate, degradation_rate)
Ah[k+1]    = Ah[k] + abs(I[k])*dt/3600
D[k+1]     = D[k] + kdeg[k]*abs(I[k])*dt/3600
SoH[k]     = min(1, max(0, 1-D[k]))
Ah[0]      = 0
D[0]       = 1-EV_SOH0
```

The lookup coefficient kdeg has units **fractional capacity loss per Ah
throughput (1/Ah)**. C-rate uses the conventional C units (numerically A/Ah).
D and SoH are dimensionless fractions. The actual instantaneous loss rate is
`capacity_loss_rate = kdeg*abs(I)/3600`, in 1/s. Both coefficient and actual
loss rate are logged and plotted to avoid confusing these two quantities.
Zero current gives zero cycling loss even if a future curve has a nonzero
zero-C coefficient. Positive and negative currents of equal magnitude cause
the same degradation. D remains cumulative even after SoH reaches zero.

The 1-D lookup uses linear interpolation and clips to endpoint values outside
the tabulated range. Therefore kdeg increases with C-rate on the supplied
strictly increasing curve, but plateaus above 4 C. This avoids extrapolating
an uncalibrated aging curve.

Editable SoH-related parameters in `initialize_model.m`:

| Parameter | Default | Meaning |
|---|---|---|
| `degradation_Crate` | `[0 0.5 1 2 3 4]` | C-rate breakpoints, starting at zero and strictly increasing |
| `degradation_scale` | `1e-6` | Scale in 1/Ah applied to the example relative curve |
| `degradation_rate` | `degradation_scale*[0 1 1.2 1.8 2.8 4]` | Absolute loss/Ah table; may instead be supplied directly |
| `EV_SOH0` | `1.0` | Initial SoH fraction in [0,1] |
| `EV_capacity_Ah` | `150` | Nominal Ah capacity, also used by existing SOC and EFC calculations |
| `EV_dt` | `0.1` | Sampling and forward Euler integration interval, seconds |
| `EV_SOC0` | `0.80` | Initial SOC, independent of SoH |

Curve vectors must have equal lengths, finite entries, and nonnegative,
nondecreasing degradation rates. The initialization script checks these rules.
`degradation_scale` affects the model through `degradation_rate`: rerun the
script after editing it, or explicitly recompute the table in the workspace.
To change from relative data to absolute measured loss/Ah data, replace the
assignment to `degradation_rate`; the subsystem interface stays the same.
The curve is illustrative and requires calibration before interpreting SoH
as a battery life prediction. No temperature, SOC-dependent or calendar aging
is included.

Run `run_ev_backward` to simulate, verify monotonicity and zero-current behavior,
save the results, and call the consolidated `plot_results` script. Battery and
degradation panels show SOC, SoH, current, C-rate, kdeg, dD/dt, accumulated
degradation, Ah throughput and equivalent full cycles. Set
`plot_export_folder = 'results'` to export PNG/FIG files; otherwise figures are
interactive only. The SoH axis is zoomed to make its small single-cycle change
visible. Additional component scenarios are in
`battery_soh.feature` for the Simulink `model_test` tool.

The added logged signals are `battery_Crate`, `degradation_rate`,
`capacity_loss_rate`, `accumulated_degradation`, and `SOH`; existing
`battery_current`, `Ah_throughput` and `SOC` logs are retained.

Validated with the existing 160 s cycle: final SoH **0.999998731850121**,
accumulated loss **1.268149879e-6**, and peak C-rate **1.472261 C**.
All cycle assertions passed: nonincreasing SoH, nondecreasing D, bounds,
lookup consistency and zero cycling loss at zero current. All eight component
scenarios passed in both draft and full-compilation modes (27 assessments).
The 14 original signals other than EFC were bit-for-bit unchanged from the
pre-edit run; EFC changed only by floating-point rounding (maximum 8.67e-19)
because it now uses the explicit capacity input. The original electrical and
SOC behavior is preserved. Connectivity checks also passed.
