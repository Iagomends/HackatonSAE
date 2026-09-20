# Planetary e-CVT operating-point controller

Run `run_ev_ecvt` for the default degradation-minimizing controller and
constraint/battery integration checks. Run `run_ecvt_comparison` to simulate
all three strategies on the same prescribed cycle and save:

- `EV_eCVT_comparison.mat`: simulation outputs, labels and per-strategy tables.
- `EV_eCVT_strategy_1.csv`, `_2.csv`, `_3.csv`: time-aligned selected points.

For interactive simulation, run `initialize_model`, set `EV_topology=2`, then
set `ecvt_strategy` to 1 (minimum normalized motor speed), 2 (minimum absolute
battery current), or 3 (minimum instantaneous battery degradation, default).
Topology 1 retains the original single-motor fixed-gear model.

## Mechanical constraints

Motor 1 drives the sun, Motor 2 the carrier, and the ring drives the wheels.
For `a=N_s/N_r`, `b=1+a`, the ideal, quasi-static planetary obeys:

```text
a*ws + wr = b*wc
T1 = -a*Tr; T2 = b*Tr
Pm1 = T1*ws; Pm2 = T2*wc
Pm1 + Pm2 = Tr*wr
```

The final-drive equations are unchanged: `wr=final_ratio*wheel_speed` and
`Tr=wheel_torque/(final_ratio*eta_final)` in motoring, or
`Tr=wheel_torque*eta_final/final_ratio` in regeneration. Power sign determines
the mode, including reverse travel. For a fixed ring request, motor torques
are fixed by equilibrium; the controller selects internal carrier speed.
Every candidate respects each motor's torque, mechanical power and speed
limits, including its generating torque/power derating.

The solver intersects the feasible carrier-speed intervals for all four
shaft-speed sign combinations. With the existing directional constant
motor efficiencies, battery power is affine in carrier speed in each interval.
It also excludes demands above the existing battery's real-current limit
`Voc^2/(4*R)` when `R>0`.

An impossible request retains the existing fallback: cap ring speed at its
kinematic limit, then use 55 bisection iterations to find the largest feasible
torque fraction. `ecvt_feasible=0` and torque/speed shortfalls explicitly mark
unserved demand. Delivered power is logged; the prescribed cycle is unchanged.

## Candidate evaluation and objective

`ecvt_candidate_metrics.m` evaluates every candidate using the existing laws:

```text
eta_i = EV_motor_eta if Pmi >= 0, otherwise EV_regen_eta
Pei = Pmi/eta_i      if Pmi >= 0, otherwise Pmi*eta_i
Pbat = Pe1 + Pe2
Ibat = 2*Pbat / (Voc + sqrt(Voc^2 - 4*R*Pbat))
C = abs(Ibat)/Qnom
kdeg = linear lookup(C), with clipped endpoints             [1/Ah]
Ddot = kdeg * abs(Ibat)/3600                                [1/s]
```

The battery current formula also works at zero resistance. The efficiencies
are selected from the candidate torque/speed power direction. No efficiency
map exists in this model, so no new speed-dependent losses or maps are invented.
The vehicle, battery, SOC and aging integration laws and numerical parameters
are unchanged.

| Strategy | Primary objective J | Secondary objective |
|---|---|---|
| 1 | `(ws/w1max)^2 + (wc/w2max)^2` | None |
| 2 | `abs(Ibat)` | Normalized speed |
| 3 (default) | `Ddot` | Normalized speed |

The candidate set contains interval endpoints, the projected unconstrained
speed minimum, zero-battery-power crossings, and aging lookup breakpoint
crossings. This finds a global optimum for the existing constant-efficiency,
nonnegative nondecreasing aging model, including flat regions. It is not a
coarse speed grid. Only numerical ties (64 floating-point spacings of the
objective magnitude) use the secondary objective; no weighted speed penalty
is added to battery degradation. A future efficiency-map model would require
revisiting this affine-interval search.

Because the existing aging curve is nondecreasing in absolute C-rate,
strategies 2 and 3 normally coincide. Minimizing degradation alone can favor
internal power circulation during braking to reduce charging current. Thus
less degradation does not necessarily mean more energy recovered or higher
final SOC. No energy-recovery preference has been added to the requested objective.

## Interfaces, parameters and logging

The transmission's three external outputs still represent the equivalent
ring shaft: mechanical power, torque and speed. The electrical adapter
reconstructs the same deterministic selected point, so efficiencies are
applied exactly once. Both controller and adapter receive the same strategy,
battery parameters and aging curve directly from workspace expressions;
`SimulationInput` overrides of these parameters are respected.

`ecvt_parameters` for direct MATLAB calls has entries 1:12 unchanged (sun/ring
teeth, torque limits, power limits, speed limits, motoring/generating efficiencies,
and generating fractions). Entries 13:17 are strategy, Voc, R, Qnom and lookup
length; the C-rate breakpoints and loss/Ah coefficients follow. Update this
vector as well when overriding parameters for direct function calls.

`ecvt_operating_point` returns this 24-element column vector:

| Indices | Contents |
|---|---|
| 1:3 | Delivered ring torque, speed, mechanical power |
| 4:7 | Motor 1/2 speeds, Motor 1/2 torques |
| 8:12 | Motor 1/2 mechanical powers, Motor 1/2 electrical powers, battery power |
| 13:18 | J, feasible flag, requested torque/speed, torque/speed shortfalls |
| 19:24 | Motor 1/2 efficiencies, signed battery current, absolute C-rate, kdeg, Ddot |

Existing motor and planetary logs retain their names. Added logs are
`motor1_efficiency`, `motor2_efficiency`, `ecvt_battery_current`,
`ecvt_battery_Crate`, `ecvt_loss_per_Ah`, and `ecvt_degradation_rate`.
The original `degradation_rate` log remains the lookup coefficient in `1/Ah`;
`ecvt_degradation_rate` is the instantaneous rate in `1/s`.
Each comparison CSV includes selected torques, speeds, efficiencies,
mechanical/electrical powers, battery current, C-rate, kdeg, Ddot, objective,
and feasibility at every simulation sample. Full SimulationOutput objects
also retain SOC, SOH, accumulated degradation and all other model logs.

## Validation

`run_ev_ecvt` checks planetary kinematics, torque/power equilibrium, final-drive
power, all motor limits, electrical conversion, battery current, C-rate,
lookup coefficient, instantaneous degradation, and SOC/Ah/SOH integration.
`check_ecvt_limits` checks all three strategies across 84 combinations of
requests and regeneration limits against independent 40,001-point speed
sweeps, plus zero resistance and lookup clipping. Requests include reverse
travel, standstill, motoring, braking, torque saturation and overspeed.

The default 160-second cycle has 1,601 samples and is fully feasible for all
three strategies. Speed minimization gives final SOH 0.9999987319; current
and degradation minimization both give 0.9999989346 (about 16% less accumulated
modeled degradation). These are illustrative model results, not measured
battery-life predictions.
