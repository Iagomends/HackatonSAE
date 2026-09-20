# Battery-objective controller validation

Validated in MATLAB/Simulink on 2026-09-19.

- `check_ecvt_limits`: 84 operating requests spanning all three strategies,
  full and reduced regeneration limits, reverse travel, standstill, torque
  saturation and overspeed. Passed physical constraints and independent
  40,001-point feasible-speed objective sweeps. Zero-resistance current and
  clipped aging lookup checks also passed.
- `run_ev_ecvt`: all 1,601 default-cycle samples feasible; planetary and
  final-drive constraints, motor envelopes, electrical adapter, battery
  current, C-rate, lookup, Ddot and SOC/Ah/SOH integration checks passed.
- `run_ecvt_comparison`: all three complete simulations passed. Current and
  degradation strategies selected equivalent battery currents; degradation
  was no worse than speed minimization at every sample.
- Pre-change topology-1 SimulationOutput compared with the updated model:
  all 17 original signal names retained and data bit-for-bit identical.
- `model_check`, root and planetary scope: healthy, no unconnected ports or
  lines. MATLAB Code Analyzer: zero messages in the solver, candidate metrics,
  adapter, comparison runner and constraint validation script.

| Strategy | Final SOH | Peak absolute current | Feasible |
|---|---:|---:|---:|
| Minimum normalized speed | 0.9999987319 | 220.839144 A | 100% |
| Minimum absolute current | 0.9999989346 | 220.839144 A | 100% |
| Minimum degradation | 0.9999989346 | 220.839144 A | 100% |

Accumulated degradation decreased by 15.986645%. Final SOC for the degradation
strategy was 0.79324277, versus 0.79494747 for speed minimization. Reduced
charging current during braking sacrifices recovered energy: degradation is
the sole primary objective. Existing directional constant efficiencies and
the existing nonnegative nondecreasing loss/Ah curve were preserved.

Regenerable MAT/CSV comparison artifacts remain in the workspace and are
excluded from version control, consistent with the existing run outputs.
