% PLOT_EV_SOH Plot the logged cycle; run run_ev_backward first.
% Lookup coefficient [1/Ah] and actual loss rate [1/s] are distinct quantities.
EV_plot_names = {'SOC','SOH','battery_current','battery_Crate', ...
    'degradation_rate','capacity_loss_rate'};
EV_plot_titles = {'State of charge','State of health', ...
    'Signed battery current (+ discharge, - charge)','Absolute C-rate', ...
    'Lookup degradation coefficient','Instantaneous capacity loss rate'};
EV_plot_units = {'SOC (%)','SoH (%)','Current (A)','C-rate (C)', ...
    'k_{deg} (1/Ah)','dD/dt (1/s)'};
EV_fig = figure('Name','Battery SOC and SoH','Color','w', ...
    'Position',[100 80 1150 900]);
EV_layout = tiledlayout(EV_fig,3,2,'TileSpacing','compact','Padding','compact');
EV_axes = gobjects(6,1);
for k = 1:6
    EV_axes(k) = nexttile(EV_layout);
    EV_trace = out.logsout.get(EV_plot_names{k}).Values;
    EV_y = EV_trace.Data;
    if k<=2
        EV_y = 100*EV_y;
    end
    plot(EV_axes(k),EV_trace.Time,EV_y,'LineWidth',1.5);
    title(EV_axes(k),EV_plot_titles{k});
    xlabel(EV_axes(k),'Time (s)');
    ylabel(EV_axes(k),EV_plot_units{k});
    grid(EV_axes(k),'on');
    xlim(EV_axes(k),[EV_trace.Time(1) EV_trace.Time(end)]);
end
EV_axes(2).YAxis.Exponent = 0;
ytickformat(EV_axes(2),'%.5f');
title(EV_layout,'C-rate-dependent cycling degradation | illustrative curve');
subtitle(EV_layout,'SoH axis is zoomed to show the small loss over one cycle');
linkaxes(EV_axes,'x');
exportgraphics(EV_fig,'EV_Battery_SoH.png','Resolution',160);
savefig(EV_fig,'EV_Battery_SoH.fig');
