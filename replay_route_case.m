function data = replay_route_case(c,plan,caseNumber,initialSOC)
%REPLAY_ROUTE_CASE Fast deterministic sensitivity screening of the controller.
% Uses the production dispatch and the existing forward-Euler battery/aging
% laws. Representative results are cross-checked against the Simulink model.
N=numel(c.time); y=zeros(N,39); soc=zeros(N,1); loss=soc; soc(1)=initialSOC;
for k=1:N
    y(k,:)=route_powertrain([plan.torque(k);plan.wheelSpeed(k); ...
        soc(k);c.time(k);caseNumber],c,plan)';
    if k<N
        soc(k+1)=soc(k)-y(k,21)*c.dt/(3600*c.p(16));
        loss(k+1)=loss(k)+y(k,24)*c.dt;
    end
end
assert(all(soc>=0 & soc<=1),'Sensitivity route exceeds physical SOC bounds.');
data=table(c.time,c.speed,soc,1-loss,loss,y(:,12),y(:,21),y(:,22), ...
    y(:,24),y(:,25),y(:,26),y(:,29),y(:,30),y(:,31),y(:,32), ...
    plan.power,y(:,38),y(:,37),y(:,39),'VariableNames',{'time_s','vehicle_speed', ...
    'SOC','SOH','accumulated_degradation','battery_power','battery_current', ...
    'battery_Crate','capacity_loss_rate','regen_fraction','friction_brake_power', ...
    'route_progress','remaining_distance','SOC_required_to_finish', ...
    'terminal_SOC_feasible','wheel_power','SOC_needed_without_regen','route_power_feasible', ...
    'route_energy_infeasible'});
data.distance_km=plan.distance/1000;
energy=@(x) [0;cumsum(x(1:end-1)*c.dt)]/3.6e6;
data.regenerated_energy_kWh=energy(max(-y(:,12),0));
data.friction_energy_kWh=energy(y(:,26));
data.predicted_remaining_energy_kWh=plan.remainingTractionEnergy_kWh;
data.remaining_recoverable_energy_kWh=plan.remainingRecoverableEnergy_kWh;
end
