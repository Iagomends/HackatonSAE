function data = route_result_table(out,c)
%ROUTE_RESULT_TABLE Full route-mode logs with energy on the plant's time grid.
L=out.logsout; names=L.getElementNames;
keep=startsWith(names,'route_') | ismember(names,{'battery_power','battery_current', ...
    'battery_Crate','SOC','SOH','accumulated_degradation','capacity_loss_rate', ...
    'vehicle_speed','wheel_torque','wheel_speed','wheel_power','regen_fraction', ...
    'friction_brake_power','drivetrain_loss_power','motor_loss_power', ...
    'remaining_distance','SOC_required_to_finish','terminal_SOC_feasible', ...
    'power_balance_residual','regenerated_power','battery_loss_power','SOC_needed_without_regen'});
names=names(keep); time=L.get('SOC').Values.Time;
x=zeros(numel(time),numel(names));
for j=1:numel(names), x(:,j)=L.get(names{j}).Values.Data; end
 data=array2table([time x],'VariableNames',[{'time_s'} names(:)']);
% Left rectangles match the existing discrete SOC and SOH integrators.
integral=@(v) [0;cumsum(v(1:end-1).*diff(time))]/3.6e6;
data.regenerated_energy_kWh=integral(data.regenerated_power);
data.friction_energy_kWh=integral(data.friction_brake_power);
data.drivetrain_loss_kWh=integral(data.drivetrain_loss_power);
data.motor_loss_kWh=integral(data.motor_loss_power);
data.battery_loss_kWh=integral(data.battery_loss_power);
data.distance_km=[0;cumsum(abs(data.vehicle_speed(1:end-1)).*diff(time))]/1000;
data.SOC_preferred_low=c.preferredMin*ones(size(time));
data.SOC_preferred_high=c.preferredMax*ones(size(time));
% The dedicated controller flag is also reconstructed here so it remains
% available even when a simulation log sink is disabled by a model variant.
data.route_energy_infeasible=false(size(time));
if ismember('route_case',data.Properties.VariableNames)
    predictive=data.route_case==3;
    data.route_energy_infeasible=predictive & ...
        ((data.SOC<c.predictiveMin-1e-10) | ...
         (data.SOC_required_to_finish>data.SOC+1e-10));
end
end
