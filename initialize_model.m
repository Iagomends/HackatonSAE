% INITIALIZE_MODEL Parameters and prescribed input for EV_Backward_Baseline.
% Run this script, then out = sim("EV_Backward_Baseline"), then plot_results.
% Existing names and values are retained for model and harness compatibility.
% Positive electrical power/current means battery discharge. No figures or sim.

%% General simulation settings
% User-defined parameters
EV_dt = 0.1;                 % Sampling and forward Euler integration step [s]
assert(EV_dt > 0);

%% Powertrain Variant Selection
% User-defined parameters
EV_topology = 1;             % 1: single motor/fixed gear; 2: planetary e-CVT [-]
% Change this line before initialization, or override EV_topology with
% SimulationInput. Both sets of machine ratings are always computed below.
% The existing VariantControl expressions and electrical selector are retained.

%% Drive Cycle
% User-defined parameters
% Illustrative prescribed cycle; not a standardized certification cycle.
EV_cycle_time = [0 5 25 55 70 80 100 130 150 160]'; % Speed-knot times [s]
EV_cycle_speed = [0 0 15 15 0 0 25 25 0 0]';       % Speed at each knot [m/s]

% Derived parameters
EV_time = (0:EV_dt:EV_cycle_time(end))';            % Sample times [s]
EV_speed = interp1(EV_cycle_time,EV_cycle_speed,EV_time,'linear'); % Speed [m/s]
EV_drive_cycle = timeseries(EV_speed,EV_time);     % Model input [m/s] vs [s]
EV_v0 = EV_speed(1);         % Delay initial speed: zero initial acceleration [m/s]

%% Vehicle Longitudinal Dynamics
% User-defined parameters
EV_mass = 1800;              % Total vehicle mass; no rotating inertia [kg]
EV_g = 9.81;                 % Gravitational acceleration [m/s^2]
EV_rho = 1.225;              % Air density [kg/m^3]
EV_Cd = 0.29;                % Aerodynamic drag coefficient [-]
EV_area = 2.2;               % Frontal area [m^2]
EV_Crr = 0.010;              % Rolling resistance coefficient [-]
EV_grade = 0;                % Road grade; positive uphill [rad]
assert(EV_mass > 0);
% Forces remain evaluated by the existing Simulink equations.

%% Wheels and Final Drive
% User-defined parameters
EV_radius = 0.31;            % Effective wheel radius [m]
EV_final = 3;                % Intermediate/ring shaft to wheel speed ratio [-]
EV_drive_eta = 0.96;         % Combined gear and differential efficiency [-]
assert(EV_radius > 0 && EV_final > 0);
assert(EV_drive_eta > 0 && EV_drive_eta <= 1);

%% Powertrain Architecture
% User-defined parameters
EV_gear = 3;                 % Fixed-gear motor/intermediate shaft speed ratio [-]
assert(EV_gear > 0);
% Topology 1 uses EV_gear and EV_final. Topology 2 uses an ideal planetary
% gearset: Motor 1 = sun, Motor 2 = carrier, output = ring. Its scalar
% transmission interface describes the equivalent ring shaft, not Motor 1.

%% Electric Motor
% User-defined parameters
EV_motor_eta = 0.92;         % Motoring efficiency including inverter [-]
EV_regen_eta = 0.85;         % Generating efficiency including inverter [-]
vehicle_max_motor_power = 160e3; % Total installed mechanical machine rating [W]
vehicle_max_motor_torque = 900;  % Sum of machine shaft torque ratings [N*m]
motor1_size_fraction = 0.5;  % Motor 1 share of installed e-CVT ratings [-]
motor2_size_fraction = 0.5;  % Motor 2 share of installed e-CVT ratings [-]
motor1_wmax = 12000*2*pi/60; % Motor 1 shaft speed limit [rad/s]
motor2_wmax = 12000*2*pi/60; % Motor 2 shaft speed limit [rad/s]
motor1_regen_fraction = 1;  % Motor 1 generating/rated torque and power [-]
motor2_regen_fraction = 1;  % Motor 2 generating/rated torque and power [-]

% Derived parameters
single_motor1_Pmax = vehicle_max_motor_power; % Single-motor rating [W]
single_motor1_Tmax = vehicle_max_motor_torque; % Single-motor rating [N*m]
ecvt_motor1_Pmax = motor1_size_fraction*vehicle_max_motor_power; % [W]
ecvt_motor2_Pmax = motor2_size_fraction*vehicle_max_motor_power; % [W]
ecvt_motor1_Tmax = motor1_size_fraction*vehicle_max_motor_torque; % [N*m]
ecvt_motor2_Tmax = motor2_size_fraction*vehicle_max_motor_torque; % [N*m]
% Compatibility aliases reflect the topology at initialization time only.
% Model calculations use the topology-independent ratings above, so changing
% EV_topology using SimulationInput does not leave stale model parameters.
if EV_topology == 1
    motor1_Pmax = single_motor1_Pmax; % Active Motor 1 rating [W]
    motor1_Tmax = single_motor1_Tmax; % Active Motor 1 rating [N*m]
    motor2_Pmax = 0;                 % Motor 2 absent [W]
    motor2_Tmax = 0;                 % Motor 2 absent [N*m]
else
    motor1_Pmax = ecvt_motor1_Pmax;   % Active Motor 1 rating [W]
    motor1_Tmax = ecvt_motor1_Tmax;   % Active Motor 1 rating [N*m]
    motor2_Pmax = ecvt_motor2_Pmax;   % Active Motor 2 rating [W]
    motor2_Tmax = ecvt_motor2_Tmax;   % Active Motor 2 rating [N*m]
end
% The historical fixed-gear model does not enforce machine envelopes.
assert(vehicle_max_motor_power > 0 && vehicle_max_motor_torque > 0);
assert(abs(motor1_size_fraction+motor2_size_fraction-1) < 1e-12 && ...
    motor1_size_fraction > 0 && motor2_size_fraction > 0);
assert(all([EV_motor_eta EV_regen_eta] > 0) && ...
    all([EV_motor_eta EV_regen_eta] <= 1));

%% Planetary e-CVT
% User-defined parameters
N_s = 30;                   % Sun tooth count [teeth]
N_r = 78;                   % Ring tooth count; planet = (N_r-N_s)/2 [teeth]

% Derived parameters
ecvt_final_ratio = EV_final; % Ring/wheel speed ratio [-]
ecvt_final_efficiency = EV_drive_eta; % Downstream mechanical efficiency [-]
% Shared solver vector: teeth; torque [N*m], power [W], speed [rad/s] limits;
% efficiencies [-]; regeneration fractions [-]. Ordering is an API contract.
ecvt_parameters = [N_s N_r ecvt_motor1_Tmax ecvt_motor2_Tmax ...
    ecvt_motor1_Pmax ecvt_motor2_Pmax motor1_wmax motor2_wmax ...
    EV_motor_eta EV_regen_eta motor1_regen_fraction motor2_regen_fraction];
assert(N_s > 0 && N_r > N_s && mod(N_r-N_s,2) == 0);
assert(all(ecvt_parameters(3:8) > 0));
assert(all(ecvt_parameters(9:12) > 0 & ecvt_parameters(9:12) <= 1));

%% Battery
% User-defined parameters
EV_voltage = 360;            % Constant open-circuit pack voltage [V]
EV_resistance = 0.08;        % Pack internal resistance [ohm]
EV_capacity_Ah = 150;        % Nominal capacity; 54 kWh at nominal voltage [Ah]
EV_SOC0 = 0.80;              % Initial state of charge [fraction]
EV_SOC_min = 0;              % Lower SOC assertion limit [fraction]
EV_SOC_max = 1;              % Upper SOC assertion limit [fraction]
assert(EV_capacity_Ah > 0 && EV_voltage > 0 && EV_resistance >= 0);
assert(EV_SOC0 > EV_SOC_min && EV_SOC0 < EV_SOC_max);

%% Battery Degradation / SoH
% User-defined parameters
EV_SOH0 = 1.0;               % Initial state of health [fraction]
EV_Ah0 = 0;                  % Initial accumulated absolute current [Ah]
EV_SOH_min = 0;              % Lower reported SoH saturation limit [fraction]
EV_SOH_max = 1;              % Upper reported SoH saturation limit [fraction]
degradation_Crate = [0 0.5 1 2 3 4]; % Absolute C-rate breakpoints [C = A/Ah]
degradation_scale = 1e-6;    % Scale for the illustrative capacity-loss curve [1/Ah]
degradation_relative_loss = [0 1 1.2 1.8 2.8 4.0]; % Relative curve shape [-]

% Derived parameters
degradation_rate = degradation_scale*degradation_relative_loss; % Loss/Ah [1/Ah]
% Or replace degradation_rate with measured absolute loss/Ah values.
% Linear interpolation, clipped endpoints. SoH has no feedback into battery
% capacity or SOC. Initial accumulated degradation remains 1-EV_SOH0.
assert(isvector(degradation_Crate) && isvector(degradation_rate) && ...
    numel(degradation_Crate) == numel(degradation_rate) && numel(degradation_Crate) >= 2);
assert(all(isfinite(degradation_Crate)) && all(diff(degradation_Crate) > 0) && ...
    degradation_Crate(1) == 0);
assert(all(isfinite(degradation_rate)) && all(degradation_rate >= 0) && ...
    all(diff(degradation_rate) >= 0), 'Use nonnegative, nondecreasing loss/Ah.');
assert(isfinite(EV_SOH0) && EV_SOH0 >= 0 && EV_SOH0 <= 1);

%% Logging and Simulation Settings
% User-defined parameters
EV_start = 0;               % Simulation start time [s]

% Derived parameters
EV_stop = EV_time(end);     % Simulation stop time [s]
% The model stores enum/string settings: FixedStepDiscrete, fixed step EV_dt,
% SignalLogging='on', SignalLoggingName='logsout', ReturnWorkspaceOutputs='on'.
% These settings are not executable commands in this initialization script.
% Model callbacks do not rerun initialization or overwrite workspace edits.
