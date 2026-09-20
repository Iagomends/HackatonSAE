function study = study_route_soc(c,routeName,initialValues)
%STUDY_ROUTE_SOC Same production controller and deterministic battery laws.
plan=prepare_route_control(c);
study=struct('name',routeName,'config',c,'plan',plan,'initialSOC',initialValues, ...
    'method','Production-controller replay; selected cases validated in Simulink');
study.tables=cell(numel(initialValues),3); rows=zeros(numel(initialValues)*3,10);
labels=strings(numel(initialValues)*3,1); n=0;
caseLabels={'Direct','Local e-CVT','Predictive e-CVT'};
for i=1:numel(initialValues)
    for caseNumber=1:3
        d=replay_route_case(c,plan,caseNumber,initialValues(i));
        study.tables{i,caseNumber}=d; n=n+1;
        margin=d.SOC-d.SOC_required_to_finish;
        braking=d.wheel_power < -1;
        rows(n,:)=[100*initialValues(i),100*d.SOC(end),100*min(d.SOC), ...
            100*min(margin),100*margin(1),double(min(margin)<=0.005), ...
            d.regenerated_energy_kWh(end), ...
            c.dt*sum(braking(1:end-1) & d.regen_fraction(1:end-1)<0.2), ...
            c.dt*sum(braking(1:end-1) & d.regen_fraction(1:end-1)>0.8), ...
            double(d.terminal_SOC_feasible(1))];
        labels(n)=caseLabels{caseNumber};
    end
end
study.summary=array2table(rows,'VariableNames',{'Initial_SOC_pct','Final_SOC_pct', ...
    'Minimum_SOC_pct','Minimum_reserve_margin_pp','Initial_reserve_margin_pp', ...
    'Approaches_reserve','Regenerated_kWh','Low_regen_braking_s','High_regen_braking_s', ...
    'Initially_terminal_feasible'});
study.summary=addvars(study.summary,labels,'Before',1,'NewVariableNames','Case');
study.policy_difference=table('Size',[numel(initialValues) 4], ...
    'VariableTypes',{'double','double','double','string'}, ...
    'VariableNames',{'Initial_SOC_pct','Local_regen_kWh','Predictive_regen_kWh','Interpretation'});
for i=1:numel(initialValues)
    local=study.tables{i,2}; predictive=study.tables{i,3};
    study.policy_difference.Initial_SOC_pct(i)=100*initialValues(i);
    study.policy_difference.Local_regen_kWh(i)=local.regenerated_energy_kWh(end);
    study.policy_difference.Predictive_regen_kWh(i)=predictive.regenerated_energy_kWh(end);
    if abs(study.policy_difference.Local_regen_kWh(i)-study.policy_difference.Predictive_regen_kWh(i))>1e-4
        study.policy_difference.Interpretation(i)="Predictive policy changes regeneration allocation";
    else
        study.policy_difference.Interpretation(i)="Policies have equivalent regeneration on this route";
    end
end
fprintf('%s: %.3f km, reserves %.6f%% (max regen), %.6f%% (no regen).\n', ...
    routeName,plan.distance(end)/1000,100*plan.required(1),100*plan.noRegenRequired(1));
disp(study.summary);
end
