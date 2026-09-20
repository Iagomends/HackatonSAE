% RUN_ROUTE_COMPARISON Identical route, battery, SOC and total installed ratings.
% No model or vehicle parameters are tuned separately for any case.
initialize_model;
assert(abs(sum(ecvt_parameters(3:4))-vehicle_max_motor_torque)<1e-9);
assert(abs(sum(ecvt_parameters(5:6))-vehicle_max_motor_power)<1e-9);
EV_route_plan=prepare_route_control(EV_route_config);
route_comparison=struct;
route_comparison.labels={'Direct fixed gear','Local degradation e-CVT','Predictive e-CVT'};
route_comparison.route_name=EV_route_name;
route_comparison.config=EV_route_config;
route_comparison.plan=EV_route_plan;
route_comparison.outputs=cell(1,3); route_comparison.tables=cell(1,3);
for caseNumber=1:3
    in=Simulink.SimulationInput('EV_Backward_Baseline');
    in=in.setVariable('EV_route_case',caseNumber);
    in=in.setVariable('EV_topology',1+(caseNumber>1));
    in=in.setVariable('EV_route_config',EV_route_config);
    in=in.setVariable('EV_route_plan',EV_route_plan);
    out=sim(in);
    route_comparison.outputs{caseNumber}=out;
    route_comparison.tables{caseNumber}=route_result_table(out,EV_route_config);
    validate_route_result(route_comparison.tables{caseNumber},EV_route_config,caseNumber);
    writetable(route_comparison.tables{caseNumber},sprintf('EV_route_case_%d.csv',caseNumber));
end
route_comparison.summary=route_summary(route_comparison.tables, ...
    route_comparison.labels,EV_route_config);
route_comparison.degradation_breakdown=route_degradation_breakdown( ...
    route_comparison.tables,route_comparison.labels);
disp(route_comparison.summary);
disp(route_comparison.degradation_breakdown);
save('EV_route_comparison.mat','route_comparison');
writetable(route_comparison.summary,'EV_route_summary.csv');
writetable(route_comparison.degradation_breakdown,'EV_route_degradation_breakdown.csv');
plot_results;
