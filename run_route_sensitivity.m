function study = run_route_sensitivity
%RUN_ROUTE_SENSITIVITY Initial-SOC sensitivity and predictive-policy figure.
% Requested values are fixed by the study specification.
initialize_model;
study=study_route_soc(EV_route_config,EV_route_name,[0.30 0.35 0.40 0.50]);
study.route_name=EV_route_name;
save('EV_route_SOC_sensitivity.mat','study');
writetable(study.summary,'validation/route_SOC_sensitivity.csv');
writetable(study.policy_difference,'validation/route_policy_difference.csv');
plot_route_sensitivity(study);
margin=study.summary.Minimum_reserve_margin_pp;
idx=find(study.summary.Approaches_reserve>0 & study.summary.Case=="Predictive e-CVT",1,'first');
if isempty(idx)
    fprintf('No predictive case approaches the reserve threshold.\n');
else
    fprintf('First predictive reserve-sensitive case: initial SOC %.1f%%, minimum margin %.4f percentage points.\n', ...
        study.summary.Initial_SOC_pct(idx),margin(idx));
end
end

function plot_route_sensitivity(study)
% Presentation of the predictive preview signals requested for the sensitivity.
vals=study.initialSOC; colors=parula(numel(vals));
fig=figure('Name','Initial SOC sensitivity: predictive reserve','Color','w', ...
    'Tag','EVResults_sensitivity','Position',[50 50 1400 850]);
tl=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for i=1:numel(vals)
    d=study.tables{i,3}; ax=nexttile(tl); hold(ax,'on');
    yyaxis(ax,'left'); plot(ax,d.time_s/60,100*d.SOC,'Color',colors(i,:),'LineWidth',1.4); 
    plot(ax,d.time_s/60,100*d.SOC_required_to_finish,'k--','LineWidth',1.1);
    ylabel(ax,'SOC [%]'); ylim(ax,[15 55]);
    yyaxis(ax,'right'); plot(ax,d.time_s/60,d.regen_fraction,'Color',[.2 .2 .2], ...
        'LineStyle',':','LineWidth',1.1); ylabel(ax,'Regeneration fraction'); ylim(ax,[-.05 1.05]);
    title(ax,sprintf('Initial SOC %.0f%%',100*vals(i))); xlabel(ax,'Route time [min]'); grid(ax,'on');
    if i==1, legend(ax,{'SOC current','SOC required','Regen fraction'},'Location','best'); end
end
ax=nexttile(tl,5); hold(ax,'on');
for i=1:numel(vals), d=study.tables{i,3}; plot(ax,d.time_s/60,100*d.route_progress,'Color',colors(i,:)); end
xlabel(ax,'Route time [min]'); ylabel(ax,'Route progress [%]'); title(ax,'Route progress'); grid(ax,'on');
ax=nexttile(tl,6); hold(ax,'on');
d=study.tables{1,3}; yyaxis(ax,'left'); plot(ax,d.time_s/60,d.wheel_power/1000,'LineWidth',1.2); ylabel(ax,'Wheel power [kW]');
yyaxis(ax,'right'); plot(ax,d.time_s/60,d.predicted_remaining_energy_kWh,'LineWidth',1.2); ylabel(ax,'Predicted remaining demand [kWh]');
xlabel(ax,'Route time [min]'); title(ax,'Route demand forecast'); grid(ax,'on');
sgtitle(tl,'Predictive reserve sensitivity: low SOC activates regeneration only when needed');
if isgraphics(fig), exportgraphics(fig,'validation/route_SOC_sensitivity.png','Resolution',140); end
end
