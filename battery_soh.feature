# --- front-matter:toml ---
model = "EV_Backward_Baseline.slx"
component = "EV_Backward_Baseline/Battery Degradation"
[inputs]
Current = "Battery Current"
Capacity = "Battery Capacity"
[outputs]
Crate = "C-rate"
Ah = "Ah Throughput"
Rate = "Degradation Rate"
Damage = "Accumulated Degradation"
Health = "SoH"
# --- end front-matter ---

Feature: C-rate dependent cycling degradation
  Tests use the initialization script's default curve and initial health.

Scenario: Zero current causes no cycling loss
  Given inputs
    * Current = const(0)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * ZeroCrate: Crate == 0
    * ZeroRate: Rate == 0
    * ZeroThroughput: Ah == 0
    * ZeroDamage: Damage == 0
    * InitialHealth: Health == 1

Scenario: Half C discharge
  Given inputs
    * Current = const(75)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * HalfC: Crate == 0.5
    * HalfCRate: Rate == [0.000000999999 .. 0.000001000001]
    * PositiveDamage: Damage > 0 when t > 0.1s
    * BoundedHealth: Health == [0 .. 1]

Scenario: Higher C discharge has higher loss per Ah
  Given inputs
    * Current = const(300)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * TwoC: Crate == 2
    * HigherRate: Rate == [0.000001799999 .. 0.000001800001]
    * PositiveDamage: Damage > 0 when t > 0.1s
    * BoundedHealth: Health == [0 .. 1]

Scenario: Charging contributes equally at equal absolute current
  Given inputs
    * Current = const(-300)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * TwoC: Crate == 2
    * ChargeRate: Rate == [0.000001799999 .. 0.000001800001]
    * ChargeThroughput: Ah > 0 when t > 0.1s
    * ChargeDamage: Damage > 0 when t > 0.1s
    * BoundedHealth: Health == [0 .. 1]

Scenario: Interpolation between breakpoints
  Given inputs
    * Current = const(225)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * OneAndHalfC: Crate == 1.5
    * InterpolatedRate: Rate == [0.000001499999 .. 0.000001500001]

Scenario: Upper endpoint is clipped
  Given inputs
    * Current = const(750)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * FiveC: Crate == 5
    * ClippedRate: Rate == [0.000003999999 .. 0.000004000001]

Scenario: Capacity input sets C-rate
  Given inputs
    * Current = const(75)
    * Capacity = const(75)
  When simulate for 2s in Normal mode
  Then outputs
    * OneC: Crate == 1
    * OneCRate: Rate == [0.000001199999 .. 0.000001200001]

Scenario: Exhausted health remains bounded
  This intentionally extreme isolated input exercises the lower saturation.
  Given inputs
    * Current = const(2000000000)
    * Capacity = const(150)
  When simulate for 2s in Normal mode
  Then outputs
    * BoundedHealth: Health == [0 .. 1]
    * ExhaustedHealth: Health == 0 when t > 1s
    * DamageBeyondUnity: Damage > 1 when t > 1s
