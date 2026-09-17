% RUN_EV_BACKWARD Initialize, simulate, and check the backward EV baseline.
initialize_model;
in = Simulink.SimulationInput('EV_Backward_Baseline');
out = sim(in);
EV_logs = out.logsout;
EV_names = EV_logs.getElementNames;
for k = 1:numel(EV_names)
    assert(all(isfinite(EV_logs.get(EV_names{k}).Values.Data(:))), ...
        'Nonfinite logged signal: %s', EV_names{k});
end
EV_I = EV_logs.get('battery_current').Values.Data;
EV_P = EV_logs.get('battery_power').Values.Data;
EV_soc = EV_logs.get('SOC').Values.Data;
EV_Ah = EV_logs.get('Ah_throughput').Values.Data;
EV_efc = EV_logs.get('equivalent_full_cycles').Values.Data;
assert(any(EV_I>0) && any(EV_I<0),'Demo must exercise motoring and regen.');
assert(max(abs(EV_P-(EV_voltage-EV_resistance*EV_I).*EV_I))<1e-6, ...
    'Battery terminal power balance failed.');
assert(all(diff(EV_Ah)>=-1e-12),'Throughput must be nondecreasing.');
assert(abs(EV_Ah(end)-EV_dt*sum(abs(EV_I(1:end-1)))/3600)<1e-8);
assert(abs(EV_soc(end)-(EV_SOC0-EV_dt*sum(EV_I(1:end-1))/ ...
    (3600*EV_capacity_Ah)))<1e-8);
assert(max(abs(EV_efc-EV_Ah/(2*EV_capacity_Ah)))<1e-12);
assert(all(EV_soc>=0 & EV_soc<=1));
EV_Pwheel = EV_logs.get('wheel_power').Values.Data;
EV_Pmotor = EV_logs.get('motor_mechanical_power').Values.Data;
EV_Pelec = EV_logs.get('motor_electrical_power').Values.Data;
assert(all(EV_Pmotor>=EV_Pwheel-1e-8),'Transmission must dissipate power.');
assert(all(EV_Pelec>=EV_Pmotor-1e-8),'Motor must dissipate power.');
EV_Crate = EV_logs.get('battery_Crate').Values.Data;
EV_kdeg = EV_logs.get('degradation_rate').Values.Data;
EV_loss_rate = EV_logs.get('capacity_loss_rate').Values.Data;
EV_D = EV_logs.get('accumulated_degradation').Values.Data;
EV_soh = EV_logs.get('SOH').Values.Data;
assert(all(diff(EV_soh)<=1e-14),'SoH must never increase.');
assert(all(diff(EV_D)>=-1e-14),'Accumulated degradation must never decrease.');
assert(all(EV_soh>=0 & EV_soh<=1),'SoH must remain in [0,1].');
assert(abs(EV_soh(1)-EV_SOH0)<1e-14);
assert(max(abs(EV_Crate-abs(EV_I)/EV_capacity_Ah))<1e-12);
EV_expected_k = interp1(degradation_Crate,degradation_rate, ...
    min(max(EV_Crate,degradation_Crate(1)),degradation_Crate(end)),'linear');
assert(max(abs(EV_kdeg-EV_expected_k))<1e-14,'Lookup interpolation mismatch.');
[~,EV_order] = sort(EV_Crate);
assert(all(diff(EV_kdeg(EV_order))>=-1e-14),'Rate must be nondecreasing with C-rate.');
assert(any(EV_I==0),'The demo must exercise zero-current intervals.');
assert(all(EV_loss_rate(EV_I==0)==0),'Zero current must produce zero cycling loss.');
EV_dD = diff(EV_D);
assert(all(abs(EV_dD(EV_I(1:end-1)==0))<1e-14));
assert(max(abs(EV_dD-EV_dt*EV_loss_rate(1:end-1)))<1e-14);
assert(max(abs(EV_soh-min(max(1-EV_D,0),1)))<1e-14);
save('EV_Backward_results.mat','out');
fprintf('PASS: %d logged signals; %.1f s demonstration cycle.\n', ...
    numel(EV_names),EV_stop);
fprintf('Final SOC %.6f; throughput %.6f Ah; EFC %.8f.\n', ...
    EV_soc(end),EV_Ah(end),EV_efc(end));
fprintf('Battery current range: %.3f to %.3f A.\n',min(EV_I),max(EV_I));
fprintf('Final SoH %.10f; accumulated loss %.10g; peak C-rate %.6f C.\n', ...
    EV_soh(end),EV_D(end),max(EV_Crate));
plot_results;
