function breakdown = route_degradation_breakdown(tables,labels)
%ROUTE_DEGRADATION_BREAKDOWN Separate discharge/charge aging contributions.
% Uses the same logged battery current, C-rate lookup output, and left
% rectangle integration convention as route_result_table.
n=numel(tables); rows=zeros(n,10);
for k=1:n
    d=tables{k}; dt=[diff(d.time_s);0]; I=d.battery_current;
    cr=abs(I)./max(eps,150); % overwritten below when nominal capacity is available
    if ismember('battery_Crate',d.Properties.VariableNames), cr=d.battery_Crate; end
    lossRate=d.capacity_loss_rate;
    dDis=sum(max(I,0).*lossRate./max(abs(I),eps).*dt);
    dReg=sum(max(-I,0).*lossRate./max(abs(I),eps).*dt);
    % At I=0 the ratio is irrelevant and the contribution is zero.
    dDis(~isfinite(dDis))=0; dReg(~isfinite(dReg))=0;
    ahDis=sum(max(I,0).*dt)/3600;
    ahReg=sum(max(-I,0).*dt)/3600;
    rows(k,:)=[dDis,dReg,dDis+dReg,ahDis,ahReg,mean(cr),sqrt(mean(cr.^2)), ...
        max([0;cr(I<0)]),max([0;cr(I>0)]),d.accumulated_degradation(end)-d.accumulated_degradation(1)];
end
breakdown=table(string(labels(:)),rows(:,1)*1e6,rows(:,2)*1e6,rows(:,3)*1e6, ...
    rows(:,4),rows(:,5),rows(:,6),rows(:,7),rows(:,8),rows(:,9),rows(:,10)*1e6, ...
    'VariableNames',{'Strategy','D_discharge_ppm','D_regen_ppm','D_total_ppm', ...
    'Ah_discharge','Ah_regen','Average_abs_Crate','RMS_Crate', ...
    'Peak_charge_Crate','Peak_discharge_Crate','Logged_total_ppm'});
assert(all(abs(breakdown.D_total_ppm-(breakdown.D_discharge_ppm+breakdown.D_regen_ppm))<1e-9), ...
    'EV:DegradationSplit','D_total must equal D_discharge + D_regen.');
end
