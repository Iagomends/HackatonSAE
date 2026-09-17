% VERIFY_MODEL_REORGANIZATION Compare both variants with pre-refactor results.
% Run from the project folder using the original default parameter values.
% The reference was captured before editing scripts or block parameters.
initialize_model;
reorg_reference = load(fullfile(fileparts(mfilename('fullpath')), ...
    'validation','reorganization_baseline.mat'));

%% Original parameter values
reorg_names = fieldnames(reorg_reference.reorgOriginalParameters);
for reorg_index = 1:numel(reorg_names)
    reorg_name = reorg_names{reorg_index};
    assert(isequaln(eval(reorg_name), ...
        reorg_reference.reorgOriginalParameters.(reorg_name)), ...
        'Original default parameter changed: %s',reorg_name);
end
fprintf('PASS: %d original initialization variables are identical.\n',numel(reorg_names));

%% Complete simulation traces, including inactive/active variant differences
reorg_report = zeros(2,6);
for reorg_topology = 1:2
    in = Simulink.SimulationInput('EV_Backward_Baseline');
    in = in.setVariable('EV_topology',reorg_topology);
    out = sim(in);
    reorg_before = reorg_reference.reorgBefore{reorg_topology};
    reorg_names = reorg_before.logsout.getElementNames;
    assert(isequal(sort(reorg_names),sort(out.logsout.getElementNames)), ...
        'Logged signal names changed for topology %d.',reorg_topology);
    assert(isequaln(reorg_before.tout,out.tout),'Simulation time grid changed.');
    reorg_error = 0;
    for reorg_index = 1:numel(reorg_names)
        reorg_name = reorg_names{reorg_index};
        reorg_old = reorg_before.logsout.get(reorg_name).Values;
        reorg_new = out.logsout.get(reorg_name).Values;
        assert(isequaln(reorg_old.Time,reorg_new.Time), ...
            'Time vector changed: topology %d / %s',reorg_topology,reorg_name);
        assert(isequaln(reorg_old.Data,reorg_new.Data), ...
            'Signal data changed: topology %d / %s',reorg_topology,reorg_name);
        reorg_error = max(reorg_error,max(abs(double(reorg_old.Data(:))- ...
            double(reorg_new.Data(:)))));
    end
    reorg_report(reorg_topology,:) = [reorg_topology,numel(reorg_names), ...
        numel(out.tout),reorg_error,out.logsout.get('SOC').Values.Data(end), ...
        out.logsout.get('SOH').Values.Data(end)];
end
reorg_report = array2table(reorg_report,'VariableNames', ...
    {'Topology','Signals','Samples','MaxAbsoluteDifference','FinalSOC','FinalSOH'});
disp(reorg_report);

%% Block inventory, layout and all dialog parameters
% Only these four block parameter expressions were changed intentionally.
reorg_allowed = {'118.Expr','122.InitialCondition','155.UpperLimit','155.LowerLimit'};
reorg_blocks = find_system('EV_Backward_Baseline', ...
    'MatchFilter',@Simulink.match.allVariants,'Type','Block');
assert(numel(reorg_blocks) == numel(reorg_reference.reorgBlockSnapshot), ...
    'Block inventory changed.');
for reorg_index = 1:numel(reorg_reference.reorgBlockSnapshot)
    reorg_snapshot = reorg_reference.reorgBlockSnapshot(reorg_index);
    reorg_block = Simulink.ID.getFullName(['EV_Backward_Baseline:' reorg_snapshot.SID]);
    assert(isequal(get_param(reorg_block,'Position'),reorg_snapshot.Position), ...
        'Block position changed: %s',reorg_snapshot.SID);
    assert(strcmp(get_param(reorg_block,'Parent'),reorg_snapshot.Parent));
    assert(strcmp(get_param(reorg_block,'BlockType'),reorg_snapshot.BlockType));
    reorg_fields = fieldnames(reorg_snapshot.Parameters);
    for reorg_field_index = 1:numel(reorg_fields)
        reorg_field = reorg_fields{reorg_field_index};
        if ismember([reorg_snapshot.SID '.' reorg_field],reorg_allowed), continue; end
        assert(isequaln(get_param(reorg_block,reorg_field), ...
            reorg_snapshot.Parameters.(reorg_field)), ...
            'Unexpected dialog change: %s.%s',reorg_snapshot.SID,reorg_field);
    end
end
fprintf('PASS: both variants have bit-for-bit identical time/data arrays.\n');
fprintf('PASS: block inventory, layout and variant controls are unchanged.\n');
