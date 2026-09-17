% PLOT_EV_ECVT Compare motor and battery traces from the e-CVT SimulationOutput.
figECVT = figure('Name','Planetary e-CVT','Color','w','Position',[80 50 1250 950]);
tilesECVT = tiledlayout(figECVT,4,2,'TileSpacing','compact','Padding','compact');
groups = {{'motor1_sun_speed','motor2_carrier_speed'}, ...
    {'motor1_torque','motor2_torque'}, ...
    {'motor1_mechanical_power','motor2_mechanical_power'}, ...
    {'motor1_electrical_power','motor2_electrical_power'}, ...
    {'battery_power'},{'battery_current'},{'SOC'},{'SOH'}};
labels = {'Speed (rad/s)','Torque (Nm)','Mechanical power (kW)', ...
    'Electrical power (kW)','Battery power (kW)','Battery current (A)', ...
    'SOC (%)','SoH (%)'};
scales = [1 1 1e-3 1e-3 1e-3 1 100 100];
axECVT = gobjects(8,1);
for k = 1:8
    axECVT(k) = nexttile(tilesECVT); hold(axECVT(k),'on');
    for j = 1:numel(groups{k})
        traceECVT = out.logsout.get(groups{k}{j}).Values;
        plot(axECVT(k),traceECVT.Time,traceECVT.Data*scales(k),'LineWidth',1.4);
    end
    xlabel(axECVT(k),'Time (s)'); ylabel(axECVT(k),labels{k});
    grid(axECVT(k),'on');
    if k<=4, legend(axECVT(k),{'M1 / Sun','M2 / Carrier'},'Location','best'); end
end
linkaxes(axECVT,'x'); xlim(axECVT(1),[0 EV_stop]);
axECVT(8).YAxis.Exponent=0; ytickformat(axECVT(8),'%.5f');
title(tilesECVT,'Pure-electric planetary e-CVT | prescribed drive cycle');
subtitle(tilesECVT,'Signed shaft quantities; positive electrical power = discharge. SoH axis zoomed.');
exportgraphics(figECVT,'EV_eCVT_plots.png','Resolution',160);
savefig(figECVT,'EV_eCVT_plots.fig');
