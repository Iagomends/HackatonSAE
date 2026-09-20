% PLOT_RESULTS Visualize completed EV_Backward_Baseline simulations.
% Preferred input: out = sim("EV_Backward_Baseline"). No simulation or init here.
% Also accepts ans when sim was called without assigning its return value.
% Optional comparison: comparison_outputs = {out_fixed, out_ecvt};
%                      comparison_labels = {'Fixed gear','Planetary e-CVT'};
% Optional export: plot_export_folder = 'results'; (default: no file export).
% Add/edit a panel by changing its signal names, legend, title, units and scale.

%% Logged simulation results
% Route dashboard uses the actual dispatched route powertrain logs. Legacy
% transmission diagnostic branches do not drive the battery in route mode.
if exist('route_comparison','var') && isstruct(route_comparison)
    delete(findall(groot,'Type','figure','-regexp','Tag','^EVResults_'));
    EV_figures = plotRouteDashboard(route_comparison);
    if exist('plot_export_folder','var') && strlength(string(plot_export_folder))>0
        if ~isfolder(plot_export_folder), mkdir(plot_export_folder); end
        exportgraphics(EV_figures.dashboard,fullfile(plot_export_folder,'EV_route_dashboard.png'),'Resolution',160);
        savefig(EV_figures.dashboard,fullfile(plot_export_folder,'EV_route_dashboard.fig'));
        savefig(EV_figures.summary,fullfile(plot_export_folder,'EV_route_summary.fig'));
    end
    return
end
if exist('out','var') && isa(out,'Simulink.SimulationOutput')
    EV_result = out;
elseif exist('ans','var') && isa(ans,'Simulink.SimulationOutput') %#ok<NOANS>
    EV_result = ans; %#ok<NOANS> Support the unassigned sim workflow explicitly.
else
    error('EV:MissingResults', ...
        'Run out = sim("EV_Backward_Baseline") or load a saved out before plotting.');
end
EV_plot_logs = EV_result.logsout;
EV_plot_names = EV_plot_logs.getElementNames;
EV_has_ecvt = any(strcmp(EV_plot_names,'motor1_sun_speed'));
% Replace only this script's previous figures; leave other figures untouched.
delete(findall(groot,'Type','figure','-regexp','Tag','^EVResults_'));
EV_figures = struct;

%% Vehicle Dynamics
EV_figures.vehicle = plotGroup(EV_plot_logs,'Vehicle dynamics','vehicle', {
    {'vehicle_speed'}, {'Vehicle'}, 'Prescribed vehicle speed', 'Speed [m/s]', 1;
    {'vehicle_acceleration'}, {'Vehicle'}, 'Vehicle acceleration', 'Acceleration [m/s^2]', 1;
    {'wheel_force'}, {'Required force'}, 'Required tractive force', 'Force [N]', 1});

%% Wheel Results
EV_figures.wheels = plotGroup(EV_plot_logs,'Wheel results','wheels', {
    {'wheel_speed'}, {'Wheels'}, 'Wheel angular speed', 'Speed [rad/s]', 1;
    {'wheel_torque'}, {'Required torque'}, 'Wheel torque', 'Torque [N m]', 1;
    {'wheel_power'}, {'Required power'}, 'Wheel mechanical power', 'Power [kW]', 1e-3});

%% Motor Results
% Read topology from the completed logs, not today's workspace selector.
% A fixed-gear run has one motor; do not invent zero traces for Motor 2.
if EV_has_ecvt
    EV_motor_panels = {
        {'motor1_sun_speed','motor2_carrier_speed'}, {'Motor 1 / Sun','Motor 2 / Carrier'}, 'Motor shaft speeds', 'Speed [rad/s]', 1;
        {'motor1_torque','motor2_torque'}, {'Motor 1','Motor 2'}, 'Motor shaft torques', 'Torque [N m]', 1;
        {'motor1_mechanical_power','motor2_mechanical_power'}, {'Motor 1','Motor 2'}, 'Motor mechanical powers', 'Power [kW]', 1e-3;
        {'motor1_electrical_power','motor2_electrical_power'}, {'Motor 1','Motor 2'}, 'Motor electrical powers', 'Power [kW]', 1e-3};
else
    EV_motor_panels = {
        {'motor_speed'}, {'Motor 1'}, 'Motor shaft speed', 'Speed [rad/s]', 1;
        {'motor_torque'}, {'Motor 1'}, 'Motor shaft torque', 'Torque [N m]', 1;
        {'motor_mechanical_power'}, {'Motor 1'}, 'Motor mechanical power', 'Power [kW]', 1e-3;
        {'motor_electrical_power'}, {'Motor 1'}, 'Motor electrical power', 'Power [kW]', 1e-3};
end
EV_figures.motors = plotGroup(EV_plot_logs,'Electric motor results','motors',EV_motor_panels);

%% Planetary e-CVT Results
% These signals exist only when the planetary variant was active.
if EV_has_ecvt
    EV_figures.ecvt = plotGroup(EV_plot_logs,'Planetary e-CVT results','ecvt', {
        {'motor1_sun_speed','ecvt_ring_speed','motor2_carrier_speed'}, {'Sun','Ring','Carrier'}, 'Planetary shaft speeds', 'Speed [rad/s]', 1;
        {'ecvt_degradation_rate'}, {'Ddot'}, 'Instantaneous battery degradation', 'Capacity loss [1/s]', 1;
        {'ecvt_feasible'}, {'Feasible'}, 'Request feasibility (1 = feasible)', 'Flag [0 or 1]', 1;
        {'ecvt_requested_ring_torque','ecvt_ring_torque'}, {'Requested','Delivered'}, 'Ring torque', 'Torque [N m]', 1;
        {'ecvt_requested_ring_speed','ecvt_ring_speed'}, {'Requested','Delivered'}, 'Ring speed', 'Speed [rad/s]', 1;
        {'ecvt_mechanical_power','ecvt_total_electrical_power'}, {'Ring mechanical','Total electrical'}, 'Planetary power', 'Power [kW]', 1e-3;
        {'ecvt_torque_shortfall'}, {'Unserved torque'}, 'Ring torque shortfall', 'Torque [N m]', 1;
        {'ecvt_speed_shortfall'}, {'Unserved speed'}, 'Ring speed shortfall', 'Speed [rad/s]', 1});
end

%% Battery Results
EV_figures.battery = plotGroup(EV_plot_logs,'Battery results','battery', {
    {'battery_power'}, {'Battery'}, 'Battery terminal power (+ discharge)', 'Power [kW]', 1e-3;
    {'battery_current'}, {'Battery'}, 'Battery current (+ discharge)', 'Current [A]', 1;
    {'battery_Crate'}, {'Absolute C-rate'}, 'Battery C-rate', 'C-rate [C = A/Ah]', 1;
    {'SOC'}, {'SOC'}, 'State of charge', 'SOC [%]', 100});

%% Battery Degradation / SoH
% Lookup coefficient [1/Ah] and actual capacity-loss rate [1/s] are distinct.
EV_figures.degradation = plotGroup(EV_plot_logs,'Battery degradation / SoH','degradation', {
    {'SOH'}, {'SoH'}, 'State of health (zoomed axis)', 'SoH [%]', 100;
    {'degradation_rate'}, {'Lookup coefficient'}, 'C-rate-dependent degradation coefficient', 'Loss per throughput [1/Ah]', 1;
    {'capacity_loss_rate'}, {'Capacity loss rate'}, 'Instantaneous degradation rate', 'Loss rate [1/s]', 1;
    {'accumulated_degradation'}, {'Accumulated loss'}, 'Accumulated capacity degradation', 'Capacity loss [%]', 100;
    {'Ah_throughput'}, {'Absolute throughput'}, 'Battery charge throughput', 'Throughput [Ah]', 1;
    {'equivalent_full_cycles'}, {'Equivalent full cycles'}, 'Equivalent full cycles', 'Cycles [-]', 1});

%% Powertrain Comparison
% Supply completed runs explicitly: this section never loads or simulates.
% Each run keeps its own time vector. Use the same cycle/settings for a fair
% comparison; SOC/SoH and energy differences otherwise include those changes.
if exist('comparison_outputs','var') && ~isempty(comparison_outputs)
    assert(iscell(comparison_outputs) && numel(comparison_outputs) >= 2, ...
        'comparison_outputs must be a cell array of at least two completed runs.');
    if exist('comparison_labels','var') && ~isempty(comparison_labels)
        EV_comparison_labels = cellstr(comparison_labels);
        assert(numel(EV_comparison_labels) == numel(comparison_outputs), ...
            'Provide one comparison label per completed run.');
    else
        EV_comparison_labels = arrayfun(@(k) sprintf('Run %d',k), ...
            1:numel(comparison_outputs),'UniformOutput',false);
    end
    EV_figures.comparison = plotComparison(comparison_outputs,EV_comparison_labels);
end

%% Optional figure export
if exist('plot_export_folder','var') && strlength(string(plot_export_folder)) > 0
    if ~isfolder(plot_export_folder), mkdir(plot_export_folder); end
    EV_figure_names = fieldnames(EV_figures);
    for EV_figure_index = 1:numel(EV_figure_names)
        EV_figure_name = EV_figure_names{EV_figure_index};
        EV_export_base = fullfile(plot_export_folder,['EV_' EV_figure_name]);
        exportgraphics(EV_figures.(EV_figure_name),[EV_export_base '.png'],'Resolution',160);
        savefig(EV_figures.(EV_figure_name),[EV_export_base '.fig']);
    end
end

%% Plotting helpers
function fig = plotGroup(logs,figureTitle,tag,panels)
    fig = figure('Name',figureTitle,'Tag',['EVResults_' tag], ...
        'Color','w','Position',[70 50 1100 740]);
    count = size(panels,1);
    cols = min(2,count);
    layout = tiledlayout(fig,ceil(count/cols),cols,'TileSpacing','compact','Padding','compact');
    axesHandles = gobjects(count,1);
    available = logs.getElementNames;
    for panel = 1:count
        ax = nexttile(layout);
        axesHandles(panel) = ax;
        hold(ax,'on');
        names = panels{panel,1};
        labels = panels{panel,2};
        for signal = 1:numel(names)
            assert(any(strcmp(available,names{signal})), ...
                'EV:MissingSignal','Required logged signal is missing: %s',names{signal});
            trace = logs.get(names{signal}).Values;
            if strcmp(names{signal},'ecvt_feasible')
                stairs(ax,trace.Time,trace.Data*panels{panel,5},'LineWidth',1.3);
                ylim(ax,[-0.05 1.05]); yticks(ax,[0 1]);
            else
                plot(ax,trace.Time,trace.Data*panels{panel,5},'LineWidth',1.3);
            end
        end
        formatAxes(ax,panels{panel,3},panels{panel,4},labels);
        if any(strcmp(names,'SOH'))
            ax.YAxis.Exponent = 0;
            ytickformat(ax,'%.5f');
        end
    end
    linkaxes(axesHandles,'x');
    title(layout,figureTitle);
end

function fig = plotComparison(outputs,labels)
    fig = figure('Name','Powertrain comparison','Tag','EVResults_comparison', ...
        'Color','w','Position',[70 50 1100 740]);
    layout = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
    panels = {'vehicle_speed','Prescribed speed','Speed [m/s]',1;
        'battery_power','Battery terminal power','Power [kW]',1e-3;
        'battery_current','Battery current','Current [A]',1;
        'SOC','State of charge','SOC [%]',100;
        'SOH','State of health','SoH [%]',100;
        'accumulated_degradation','Accumulated capacity loss','Capacity loss [%]',100};
    axesHandles = gobjects(size(panels,1),1);
    for panel = 1:size(panels,1)
        ax = nexttile(layout); axesHandles(panel) = ax; hold(ax,'on');
        for runIndex = 1:numel(outputs)
            assert(isa(outputs{runIndex},'Simulink.SimulationOutput'), ...
                'Each comparison entry must be a Simulink.SimulationOutput.');
            trace = outputs{runIndex}.logsout.get(panels{panel,1}).Values;
            plot(ax,trace.Time,trace.Data*panels{panel,4},'LineWidth',1.3);
        end
        formatAxes(ax,panels{panel,2},panels{panel,3},labels);
        if strcmp(panels{panel,1},'SOH')
            ax.YAxis.Exponent = 0; ytickformat(ax,'%.5f');
        end
    end
    linkaxes(axesHandles,'x');
    title(layout,'Completed powertrain simulations');
end

function formatAxes(ax,panelTitle,units,labels)
    grid(ax,'on');
    xlabel(ax,'Time [s]');
    ylabel(ax,units);
    legend(ax,labels,'Location','best','Interpreter','none');
    title(ax,panelTitle);
end


function figures = plotRouteDashboard(comparison)
    colors=[0.22 .36 .55;.90 .39 .12;.02 .52 .40];
    styles={'-','--','-.'}; labels={'Direct Drive','Local-Degradation e-CVT','Route-Aware Predictive e-CVT'};
    figures.dashboard=figure('Name','Does route preview protect the SOC reserve?', ...
        'Tag','EVResults_route','Color','w','Position',[30 40 1400 900]);
    tl=tiledlayout(figures.dashboard,3,3,'TileSpacing','compact','Padding','compact');
    d=comparison.tables{3}; t=d.time_s/60;
    ax=nexttile(tl,1); hold(ax,'on');
    plot(ax,t,d.vehicle_speed*3.6,'Color',colors(1,:),'LineWidth',1.8);
    ylabel(ax,'Speed [km/h]'); xlabel(ax,'Route time [min]'); grid(ax,'on');
    yyaxis(ax,'right');
    plot(ax,t,100*d.route_progress,':','Color',[.35 .35 .35],'LineWidth',1.5);
    plot(ax,t,100*d.regen_fraction,'-.','Color',colors(3,:),'LineWidth',1.2);
    ylabel(ax,'Progress [%] / regen fraction x100'); ylim(ax,[0 100]);
    title(ax,'1  Route speed, progress and predictive regeneration');
    legend(ax,{'Prescribed speed','Route progress','Predictive regen fraction x100'}, ...
        'Location','northwest','FontSize',9);
    yyaxis(ax,'left');
    ax=nexttile(tl,9); hold(ax,'on');
    energy=zeros(3,2);
    for k=1:3
        q=comparison.tables{k};
        energy(k,:)=[q.regenerated_energy_kWh(end),q.friction_energy_kWh(end)];
    end
    bars=bar(ax,1:3,energy,'grouped'); bars(1).FaceColor=colors(3,:); bars(2).FaceColor=[.62 .65 .69];
    xticks(ax,1:3); xticklabels(ax,{'Direct','Local e-CVT','Predictive e-CVT'});
    ylabel(ax,'Route energy [kWh]'); title(ax,'5  Recover energy only when it is useful'); grid(ax,'on');
    legend(ax,{'Returned to battery','Friction braking'},'Location','best','FontSize',9);
    ax=nexttile(tl,2,[2 2]); hold(ax,'on');
    upper=comparison.config.preferredMax*100; lower=comparison.config.preferredMin*100;
    ymin=min(cellfun(@(q) min(q.SOC)*100,comparison.tables))-1;
    ymin=min(ymin,lower-1); ymax=max(upper+2,100*comparison.config.initialSOC+2);
    patch(ax,[t(1) t(end) t(end) t(1)],[ymin ymin lower lower],[1 .92 .9], ...
        'EdgeColor','none','HandleVisibility','off');
    lines=gobjects(4,1);
    for k=1:3
        q=comparison.tables{k};
        lines(k)=plot(ax,q.time_s/60,100*q.SOC,'Color',colors(k,:), ...
            'LineStyle',styles{k},'LineWidth',2);
    end
    lines(4)=plot(ax,t,100*d.SOC_required_to_finish,'k:','LineWidth',1.8);
    yline(ax,lower,'--','20% preferred minimum','Color',[.55 .16 .12], ...
        'LabelHorizontalAlignment','left','HandleVisibility','off');
    yline(ax,upper,':','80% preferred maximum','Color',[.45 .45 .45], ...
        'LabelHorizontalAlignment','left','HandleVisibility','off');
    ylim(ax,[ymin ymax]); xlim(ax,[t(1) t(end)]);
    title(ax,'2  SOC reserve: can each strategy complete the route above 20%?');
    ylabel(ax,'SOC [%]'); xlabel(ax,'Route time [min]'); grid(ax,'on');
    legend(ax,lines,[labels {'Predictive SOC required to finish'}], ...
        'Location','northoutside','Orientation','horizontal','FontSize',9);
    finalText=sprintf('Final SOC  |  Direct %.2f%%   Local %.2f%%   Predictive %.2f%%', ...
        100*comparison.tables{1}.SOC(end),100*comparison.tables{2}.SOC(end),100*d.SOC(end));
    text(ax,.99,.88,finalText,'Units','normalized','HorizontalAlignment','right', ...
        'FontSize',10,'BackgroundColor','w','Margin',4);
    panels={'battery_Crate','3  Battery C-rate','C-rate [1/h]',1; ...
        'accumulated_degradation','4  Capacity loss accumulated on this route','Capacity loss [ppm]',1e6};
    for j=1:2
        if j==1, ax=nexttile(tl,4); else, ax=nexttile(tl,7,[1 2]); end
        hold(ax,'on');
        for k=1:3
            q=comparison.tables{k}; values=q.(panels{j,1});
            if j==2, values=values-values(1); end
            plot(ax,q.time_s/60,values*panels{j,4},'Color',colors(k,:), ...
                'LineStyle',styles{k},'LineWidth',1.7);
        end
        title(ax,panels{j,2}); ylabel(ax,panels{j,3}); xlabel(ax,'Route time [min]'); grid(ax,'on');
    end
    title(tl,sprintf('Route preview and battery aging  |  %.1f km  |  %.0f%% initial SOC', ...
        d.distance_km(end),100*comparison.config.initialSOC),'FontSize',17,'FontWeight','bold');
    figures.summary=figure('Name','Seven-metric comparison','Tag','EVResults_route_summary', ...
        'Color','w','Position',[60 100 900 330]);
    s=route_summary(comparison.tables,labels,comparison.config);
    vals=s{:,2:4}; formatted=arrayfun(@(x) sprintf('%.4f',x),vals,'UniformOutput',false);
    cells=[cellstr(s.Metric) formatted];
    uitable(figures.summary,'Data',cells,'ColumnName',{'Metric','Direct','Local e-CVT','Predictive e-CVT'}, ...
        'RowName',{},'Units','normalized','Position',[.02 .05 .96 .90], ...
        'ColumnWidth',{285,165,165,190},'FontSize',12);
end
