%% wing line connections automation (rarely needs to be modified)

% Add inboard and outboard summation + scopes

inboardLevel = model + "/Parallelogram";
outboardLevel  = model + "/Wing Tip";

%% Delete old sum/scope blocks if they exist
blocksToDelete = [ ...
    inboardLevel + "/Sum_L_in", ...
    inboardLevel + "/Sum_T_in", ...
    inboardLevel + "/Scope_Forces_In", ...
    outboardLevel  + "/Sum_L_out", ...
    outboardLevel  + "/Sum_T_out", ...
    outboardLevel  + "/Scope_Forces_Out" ];

for b = 1:length(blocksToDelete)
    blk = char(blocksToDelete(b));
    try
        delete_connected_lines(blk);   
        delete_block(blk);
    catch
    end
end

%% Create inboard sum + scope
add_block("simulink/Math Operations/Sum", char(inboardLevel + "/Sum_L_in"), ...
    'Inputs', repmat('+',1,WingSecNum_In), ...
    'Position', [900 120 930 120+20*WingSecNum_In]);

add_block("simulink/Math Operations/Sum", char(inboardLevel + "/Sum_T_in"), ...
    'Inputs', repmat('+',1,WingSecNum_In), ...
    'Position', [900 260 930 260+20*WingSecNum_In]);

add_block("simulink/Sinks/Scope", char(inboardLevel + "/Scope_Forces_In"), ...
    'Position', [1080 170 1110 230]);

set_param(char(inboardLevel + "/Scope_Forces_In"), 'NumInputPorts', '2');

%% Connect inboard segment outputs to sums
for i = 1:WingSecNum_In
    segName = "WingSeg_In_" + num2str(i);

    % output 1 = dL
    lh1 = add_line(char(inboardLevel), ...
        char(segName + "/1"), ...
        char("Sum_L_in/" + num2str(i)), ...
        'autorouting', 'on');

    set_param(lh1, 'Name', char("dL_i" + num2str(i)));

    % output 2 = dT
    lh2 = add_line(char(inboardLevel), ...
        char(segName + "/2"), ...
        char("Sum_T_in/" + num2str(i)), ...
        'autorouting', 'on');

    set_param(lh2, 'Name', char("dT_i" + num2str(i)));

    % turn on logging at source outports
    ph = get_param(char(inboardLevel + "/" + segName), 'PortHandles');

    try
        set_param(ph.Outport(1), 'DataLogging', 'on');
        set_param(ph.Outport(1), 'DataLoggingNameMode', 'Custom');
        set_param(ph.Outport(1), 'DataLoggingName', char("dL_i" + num2str(i)));
    catch
    end

    try
        set_param(ph.Outport(2), 'DataLogging', 'on');
        set_param(ph.Outport(2), 'DataLoggingNameMode', 'Custom');
        set_param(ph.Outport(2), 'DataLoggingName', char("dT_i" + num2str(i)));
    catch
    end
end

%% Connect inboard sums to scope
lh3 = add_line(char(inboardLevel), 'Sum_L_in/1', 'Scope_Forces_In/1', 'autorouting', 'on');
set_param(lh3, 'Name', 'L_tot_in');

lh4 = add_line(char(inboardLevel), 'Sum_T_in/1', 'Scope_Forces_In/2', 'autorouting', 'on');
set_param(lh4, 'Name', 'T_tot_in');

%% Create outboard sum + scope
add_block("simulink/Math Operations/Sum", char(outboardLevel + "/Sum_L_out"), ...
    'Inputs', repmat('+',1,WingSecNum_Out), ...
    'Position', [900 120 930 120+20*WingSecNum_Out]);

add_block("simulink/Math Operations/Sum", char(outboardLevel + "/Sum_T_out"), ...
    'Inputs', repmat('+',1,WingSecNum_Out), ...
    'Position', [900 260 930 260+20*WingSecNum_Out]);

add_block("simulink/Sinks/Scope", char(outboardLevel + "/Scope_Forces_Out"), ...
    'Position', [1080 170 1110 230]);

set_param(char(outboardLevel + "/Scope_Forces_Out"), 'NumInputPorts', '2');

%% Connect outboard segment outputs to sums

for i = 1:WingSecNum_Out
    segName = "WingSeg_Out_" + num2str(i);

    % output 1 = dL
    lh1 = add_line(char(outboardLevel), ...
        char(segName + "/1"), ...
        char("Sum_L_out/" + num2str(i)), ...
        'autorouting', 'on');

    set_param(lh1, 'Name', char("dL_o" + num2str(i)));

    % output 2 = dT
    lh2 = add_line(char(outboardLevel), ...
        char(segName + "/2"), ...
        char("Sum_T_out/" + num2str(i)), ...
        'autorouting', 'on');

    set_param(lh2, 'Name', char("dT_o" + num2str(i)));

    % turn on logging at source outports
    ph = get_param(char(outboardLevel + "/" + segName), 'PortHandles');

    try
        set_param(ph.Outport(1), 'DataLogging', 'on');
        set_param(ph.Outport(1), 'DataLoggingNameMode', 'Custom');
        set_param(ph.Outport(1), 'DataLoggingName', char("dL_o" + num2str(i)));
    catch
    end

    try
        set_param(ph.Outport(2), 'DataLogging', 'on');
        set_param(ph.Outport(2), 'DataLoggingNameMode', 'Custom');
        set_param(ph.Outport(2), 'DataLoggingName', char("dT_o" + num2str(i)));
    catch
    end
end

%% Connect outboard sums to scope
lh3 = add_line(char(outboardLevel), 'Sum_L_out/1', 'Scope_Forces_Out/1', 'autorouting', 'on');
set_param(lh3, 'Name', 'L_tot_out');

lh4 = add_line(char(outboardLevel), 'Sum_T_out/1', 'Scope_Forces_Out/2', 'autorouting', 'on');
set_param(lh4, 'Name', 'T_tot_out');

%% add lines to get delY_momentum 
add_line(char(outboardLevel),'Sum_L_out/1','Saturation/1','autorouting', 'on')
add_line(char(inboardLevel),'Sum_L_in/1','Saturation/1','autorouting', 'on')

add_line(char(outboardLevel),'Sum_T_out/1','Saturation1/1','autorouting', 'on')
add_line(char(inboardLevel),'Sum_T_in/1','Saturation1/1','autorouting', 'on')

disp("Inboard/outboard summation and scopes added.");
