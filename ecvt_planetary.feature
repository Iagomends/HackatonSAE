# --- front-matter:toml ---
model = "EV_Backward_Baseline.slx"
component = "EV_Backward_Baseline/Transmission/Planetary_eCVT_Two_Motors"
[inputs]
Torque = "wheel_torque"
Speed = "wheel_speed"
[outputs]
Power = "motor_mechanical_power"
RingTorque = "motor_torque"
RingSpeed = "motor_speed"
# --- end front-matter ---
Feature: Planetary eCVT backward operating points
  Uses the default initialization parameters.
Scenario: Motoring request
  Given inputs
    * Torque = const(300)
    * Speed = const(50)
  When simulate for 1s in Normal mode
  Then outputs
    * RingSpeedConstraint: RingSpeed == 150
    * RingTorqueDemand: RingTorque == [104.1666666 .. 104.1666668]
    * DriveLoss: Power == [15624.999 .. 15625.001]
Scenario: Regenerative request
  Given inputs
    * Torque = const(-300)
    * Speed = const(50)
  When simulate for 1s in Normal mode
  Then outputs
    * RingSpeedConstraint: RingSpeed == 150
    * RingTorqueDemand: RingTorque == [-96.000001 .. -95.999999]
    * RegenLoss: Power == [-14400.001 .. -14399.999]
Scenario: Torque saturation at standstill
  Given inputs
    * Torque = const(100000)
    * Speed = const(0)
  When simulate for 1s in Normal mode
  Then outputs
    * LimitedTorque: RingTorque == [324.99999 .. 325.00001]
    * ZeroSpeed: RingSpeed == 0
    * ZeroPower: Power == 0
Scenario: Kinematically impossible speed is capped
  Given inputs
    * Torque = const(0)
    * Speed = const(5000)
  When simulate for 1s in Normal mode
  Then outputs
    * BoundedSpeed: RingSpeed == [2223 .. 2224]
    * NoTorque: RingTorque == 0
    * NoPower: Power == 0
