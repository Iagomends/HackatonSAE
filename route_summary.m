function summary = route_summary(tables,~,c)
%ROUTE_SUMMARY Presentation table: seven metrics, one column per strategy.
x=zeros(7,numel(tables));
for k=1:numel(tables)
    d=tables{k}; loss=d.accumulated_degradation(end)-d.accumulated_degradation(1);
    x(:,k)=[100*d.SOC(end);100*min(d.SOC);loss*1e6; ...
        d.regenerated_energy_kWh(end);d.friction_energy_kWh(end); ...
        max(d.battery_Crate);sum(diff(d.time_s).*(d.SOC(1:end-1)<c.preferredMin-1e-10))];
end
Metric=["Final SOC [%]";"Minimum SOC [%]";"Capacity loss [ppm]"; ...
    "Regenerated energy [kWh]";"Friction brake energy [kWh]";"Peak C-rate";"Time below 20% SOC [s]"];
summary=addvars(array2table(x,'VariableNames',{'Direct','Local_eCVT','Predictive_eCVT'}), ...
    Metric,'Before',1);
end
