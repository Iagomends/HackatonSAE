% INIT_EV_BACKWARD  Editable assumptions for EV_Backward_Baseline.
% SI units unless explicitly labelled Ah. Positive power/current = discharge.
EV_dt = 0.1;                 % s, speed sampling and integration interval
EV_mass = 1800;              % kg, total vehicle mass (no rotating inertia)
EV_g = 9.81;                 % m/s^2
EV_rho = 1.225;              % kg/m^3
EV_Cd = 0.29;
EV_area = 2.2;               % m^2, frontal area
EV_Crr = 0.010;
EV_grade = 0;                % rad, positive uphill in positive travel direction
EV_radius = 0.31;            % m, effective wheel radius
EV_gear = 3;                 % motor / intermediate shaft speed
EV_final = 3;                % intermediate shaft / wheel speed
EV_drive_eta = 0.96;         % combined gear and differential efficiency
EV_motor_eta = 0.92;         % motoring efficiency, inverter included
EV_regen_eta = 0.85;         % generating efficiency, inverter included
EV_voltage = 360;            % V, constant open-circuit voltage
EV_resistance = 0.08;        % ohm, pack resistance
EV_capacity_Ah = 150;        % Ah, nominal pack capacity (54 kWh at nominal V)
EV_SOC0 = 0.80;              % fraction, initial SOC
% C-rate-dependent cycling capacity loss (illustrative, NOT calibrated).
% C-rate is |current_A| / nominal_capacity_Ah, conventionally reported in C.
degradation_Crate = [0 0.5 1 2 3 4]; % C, strictly increasing breakpoints
degradation_scale = 1e-6;     % 1/Ah, scales the example relative curve below
degradation_rate = degradation_scale * [0 1 1.2 1.8 2.8 4.0]; % 1/Ah
% Or replace degradation_rate directly with absolute loss/Ah measurements.
% Linear interpolation; endpoint clipping outside the breakpoint range.
EV_SOH0 = 1.0;               % fraction, initial health; D(0)=1-EV_SOH0
% SoH is an estimate only: it does not feed back into SOC or pack capacity.
EV_topology = 1;             % 1 = existing fixed gear; 2 = planetary e-CVT
% Installed vehicle capability. Existing single-motor blocks remain unchanged;
% their historical baseline does not enforce these newly defined envelopes.
vehicle_max_motor_power = 160e3; % W, sum of mechanical machine ratings
vehicle_max_motor_torque = 900;  % Nm, sum of machine shaft torque ratings
motor1_size_fraction = 0.5;
motor2_size_fraction = 0.5;
assert(abs(motor1_size_fraction+motor2_size_fraction-1)<1e-12 && ...
    motor1_size_fraction>0 && motor2_size_fraction>0);
motor1_wmax = 12000*2*pi/60; % rad/s
motor2_wmax = 12000*2*pi/60; % rad/s
motor1_regen_fraction = 1;  % generating torque/power relative to rated limits
motor2_regen_fraction = 1;
N_s = 30;                   % sun teeth
N_r = 78;                   % ring teeth; planet teeth=(N_r-N_s)/2=24
ecvt_final_ratio = EV_final;
ecvt_final_efficiency = EV_drive_eta; % lumped downstream mechanical efficiency
% Always derive both configurations, so topology can be changed via SimulationInput.
single_motor1_Pmax = vehicle_max_motor_power;
single_motor1_Tmax = vehicle_max_motor_torque;
ecvt_motor1_Pmax = motor1_size_fraction*vehicle_max_motor_power;
ecvt_motor2_Pmax = motor2_size_fraction*vehicle_max_motor_power;
ecvt_motor1_Tmax = motor1_size_fraction*vehicle_max_motor_torque;
ecvt_motor2_Tmax = motor2_size_fraction*vehicle_max_motor_torque;
if EV_topology == 1
    motor1_Pmax = single_motor1_Pmax; motor1_Tmax = single_motor1_Tmax;
    motor2_Pmax = 0; motor2_Tmax = 0;
else
    motor1_Pmax = ecvt_motor1_Pmax; motor1_Tmax = ecvt_motor1_Tmax;
    motor2_Pmax = ecvt_motor2_Pmax; motor2_Tmax = ecvt_motor2_Tmax;
end
% Shared e-CVT calculation parameter vector: teeth, T/P/w limits, efficiencies,
% regeneration fractions. Sizing is centralized here, not in Simulink blocks.
ecvt_parameters = [N_s N_r ecvt_motor1_Tmax ecvt_motor2_Tmax ...
    ecvt_motor1_Pmax ecvt_motor2_Pmax motor1_wmax motor2_wmax ...
    EV_motor_eta EV_regen_eta motor1_regen_fraction motor2_regen_fraction];
assert(vehicle_max_motor_power>0 && vehicle_max_motor_torque>0);
assert(N_s>0 && N_r>N_s && mod(N_r-N_s,2)==0);
assert(all(ecvt_parameters(3:8)>0));
assert(all(ecvt_parameters(9:12)>0 & ecvt_parameters(9:12)<=1));
% Illustrative cycle, NOT a standardized certification drive cycle.
% Replace these knots with any prescribed speed history (m/s).
EV_cycle_time = [0 5 25 55 70 80 100 130 150 160]';
EV_cycle_speed = [0 0 15 15 0 0 25 25 0 0]';
EV_time = (0:EV_dt:EV_cycle_time(end))';
EV_speed = interp1(EV_cycle_time,EV_cycle_speed,EV_time,'linear');
EV_drive_cycle = timeseries(EV_speed,EV_time);
EV_stop = EV_time(end);
EV_v0 = EV_speed(1);         % derivative initial condition: zero initial acceleration
assert(EV_mass>0 && EV_radius>0 && EV_dt>0);
assert(EV_gear>0 && EV_final>0 && EV_capacity_Ah>0);
assert(EV_voltage>0 && EV_resistance>=0 && EV_SOC0>0 && EV_SOC0<1);
assert(all([EV_drive_eta EV_motor_eta EV_regen_eta]>0) && ...
    all([EV_drive_eta EV_motor_eta EV_regen_eta]<=1));
assert(isvector(degradation_Crate) && isvector(degradation_rate) && ...
    numel(degradation_Crate)==numel(degradation_rate) && ...
    numel(degradation_Crate)>=2);
assert(all(isfinite(degradation_Crate)) && all(diff(degradation_Crate)>0) && ...
    degradation_Crate(1)==0);
assert(all(isfinite(degradation_rate)) && all(degradation_rate>=0) && ...
    all(diff(degradation_rate)>=0), 'Use nonnegative, nondecreasing loss/Ah.');
assert(isfinite(EV_SOH0) && EV_SOH0>=0 && EV_SOH0<=1);
