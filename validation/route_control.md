# Route-control validation

Run in MATLAB R2024b / Simulink, using the unchanged predefined route and
vehicle/battery parameters. The saved comparison is reproducible with
`run_route_comparison`; the dashboard is in `plot_results.m`.

## Default comparison

The current sensitivity route is the 1,380 second, approximately 41.8 km
motorway/service-area profile. All cases use 80% initial SOC and 160 kW /
900 Nm total installed motor ratings. No physical parameters are tuned between
cases. Run `run_route_comparison` to regenerate the current seven-metric table
and CSV outputs after changing the route profile.

The previously archived 1.962 km route showed an inactive terminal constraint;
its table is retained only as historical context in the earlier generated MAT
artifacts. It is not used as the current sensitivity scenario.

## Physical and integration checks

`validate_route_result` passed at all 1,601 samples of each case:

- Wheel/motor/battery power accounting, with explicit friction, drivetrain,
  motor and battery-ohmic losses.
- Individual motor torque, mechanical power and speed limits; generating
  derating and planetary speed/torque relationships for both e-CVT cases.
- Agreement between dispatched current/C-rate/aging and the original battery
  and degradation blocks, plus discrete SOC and accumulated-loss integration.
- Brake fractions in [0,1], nonnegative friction power, and reachable terminal
  constraints maintained throughout predictive runs.

The maximum-regeneration polytope solution was independently checked against
eight 101-by-8,001 torque-fraction/carrier-speed sweeps, including reverse
travel and reduced generation limits. All checks passed.

`model_check` reported healthy connectivity. With route mode disabled, all
17 original single-motor logged signals retained their names and bit-for-bit
identical data compared with the pre-change reference.

## Terminal feasibility and feedback tests

The unchanged route's calculated starting reserves are:

- Minimum feasible initial SOC: **0.205052533**.
- Initial SOC sufficient without any regeneration: **0.207794480**.

These auxiliary initial-SOC overrides validate the controller; they were not
used to tune or replace the default three-case comparison.

| Auxiliary predictive initial SOC | Final SOC | Initial terminal feasibility | Regenerated energy |
|---:|---:|---|---:|
| 0.205052533 (exact reachable boundary) | 0.200000000 | Feasible | 0.150070 kWh |
| 0.206423507 (midpoint of the two reserves) | 0.200000000 | Feasible | 0.074422 kWh |
| 0.200000000 (below reachable boundary) | 0.194947467 | Infeasible, correctly flagged | 0.150070 kWh |

At the same auxiliary midpoint initial SOC, the local controller finished at
**0.199666281** and spent **51.30 seconds below 20%**, confirming that no
preferred-SOC clamp or route reserve is imposed on Case 2.

Doubling the predictive grid from 301 SOC / 41 charging nodes to 601 / 81
changed midpoint-run capacity loss from 1.02216201039e-6 to 1.02214919442e-6,
or **0.001254%**. Both resolutions finished at exactly 20% to displayed
precision. The independent analytic reachability guard, rather than grid
resolution, maintains terminal feasibility.

The predictive objective is numerically approximated by the Bellman table;
the implementation does not claim an exact continuous global cost optimum.
All guarantees are conditional on the deterministic prescribed route and
the existing vehicle/battery model.
