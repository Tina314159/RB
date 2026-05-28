% This script can:
% 1. deletes previously auto-created wing segments
% 2. recreates inboard and outboard wing segments from template "WingSeg"

% see layout for xyz orientation, joints, & linkages

% example of port path for rigid transform
% F = .../RConn1 = follower
% B = .../LConn1 = base
% example of port path for solid blocks
% R = .../RConn1 = center of mass frame

% NOTE: run WingModify.m immediatly after. Some setup need to be
% corrected (ex: mass)

%% some important block path
InboardLinkages  = "Parallelogram";
OutboardLinkages = "Wing Tip";
 
templateBlk = model + "/WingSeg";
%% positioning blocks
% Optional layout positions for created blocks
b_x_in  = 300;
b_x_out = 650;
b_y0    = 100;
b_dy    = 140;
b_w     = 140;
b_h     = 90;

%% delete previously created blocks
parents = {char(model + "/Parallelogram"), char(model + "/Wing Tip")};

for p = 1:numel(parents)
    blocks = find_system(parents{p}, 'SearchDepth', 1, 'Type', 'Block');

    for k = 1:numel(blocks)
        blk = string(blocks{k});
        [~, name] = fileparts(blocks{k});

        if startsWith(name, "WingSeg_In_") || startsWith(name, "WingSeg_Out_")
            try
                delete_connected_lines(blocks{k});
                delete_block(blocks{k});
            catch ME
                disp("Failed on " + blk + ": " + ME.message);
            end
        end
    end
end
%% =========================
%% Create inboard wing segments
%% =========================
for i = 1:WingSecNum_In

    set_param(templateBlk, 'Commented', 'off');  % uncomment

    segName = "WingSeg_In_" + num2str(i);

    newBlk = model + "/" + InboardLinkages + "/" + segName;

    % Copy template subsystem
    add_block(templateBlk, newBlk, ...
        'MakeNameUnique', 'off', ...
        'CopyOption', 'duplicate');

    % Internal block paths INSIDE this copied segment
    blkSolid  = newBlk + "/WingSolid";
    blkJoint  = newBlk + "/Wing_Joint";
    blkCOMXfm = newBlk + "/Xfm";
    blkGain   = newBlk + "/pitchGain";
    blkGain1  = newBlk + "/pitchGain1";
    blkGain2  = newBlk + "/pitchGain2";
 
    % Set position of copied subsystem on canvas
    y = b_y0 + (i-1)*b_dy;
    set_param(newBlk, 'Position', [b_x_in y b_x_in+b_w y+b_h]);

    % Set index block value
    set_param(newBlk + "/i", 'Value', num2str(i));

    % Set inboard flag
    set_param(newBlk + "/In_yes", 'Value', '1');

    % ----------------------------
    % Inboard gain settings
    % ----------------------------
    set_param(blkGain,  'Gain', '0');
    set_param(blkGain1, 'Gain', '0');
    set_param(blkGain2, 'Gain', '0');
    % ----------------------------
    % Solid properties(version 2)
    % ----------------------------
    set_param(blkSolid, 'BrickDimensions', ...
        "[InboardCord ,L_t , SectionWidth_In ]");
 
    set_param(blkSolid, 'BasedOnType', 'Mass');
    set_param(blkSolid, 'MassUnits', 'kg');
    set_param(blkSolid, 'Mass', '0.01');

    set_param(blkSolid, 'GraphicDiffuseColor', ...
        "[" + num2str(i/WingSecNum_In) + " 0.5 0.5]");
 
    % ----------------------------
    % COM transform: (version 2)
    % moves from quarter-chord pivot to solid center
    % ----------------------------
    % spanwise location of section quarter-chord pivot
    zPivot = (2*i - 1) * SectionWidth_In / 2;

    % offset from quarter-chord pivot to brick center
    % assumes chord direction is +x here based on your BrickDimensions usage
    xCOM = InboardCord / 4;
    set_param(blkCOMXfm, ...
        'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', ...
        "[" + num2str(xCOM) + " 0 " + num2str(zPivot) + "]", ...
        'RotationMethod', 'None');

    % ----------------------------
    % Joint settings (optional)
    % ----------------------------
    set_param(blkJoint, 'MotionActuationMode', 'InputMotion');
    set_param(blkJoint, 'TorqueActuationMode', 'ComputedTorque');

    add_line(sprintf('%s/%s', model, InboardLinkages), ...
        'Inboard_ref/LConn2', segName + "/LConn1", 'autorouting', 'on');

    set_param(templateBlk, 'Commented', 'on');   % comment out

end

%% =========================
%% Create outboard wing segments
%% =========================
for i = 1:WingSecNum_Out
    set_param(templateBlk, 'Commented', 'off');  % uncomment

    segName = "WingSeg_Out_" + num2str(i);

    newBlk = model + "/" + OutboardLinkages + "/" + segName;

    % Copy template subsystem
    add_block(templateBlk, newBlk, ...
        'MakeNameUnique', 'off', ...
        'CopyOption', 'duplicate');

    % Internal block paths INSIDE this copied segment
    blkSolid  = newBlk + "/WingSolid";
    blkJoint  = newBlk + "/Wing_Joint";
    blkCOMXfm = newBlk + "/Xfm";
    blkGain   = newBlk + "/pitchGain";
    blkGain1  = newBlk + "/pitchGain1";
    blkGain2  = newBlk + "/pitchGain2";

    % Set position
    y = b_y0 + (i-1)*b_dy;
    set_param(newBlk, 'Position', [b_x_out y b_x_out+b_w y+b_h]);

    % Set index block value
    set_param(newBlk + "/i", 'Value', num2str(i));

    % Set outboard flag
    set_param(newBlk + "/In_yes", 'Value', '0');

    % chord dimension
    chord_i = CordOut(i);

    % spanwise location of section quarter-chord pivot
    zPivot = (2*i - 1) * SectionWidth_Out / 2;

    % offset from quarter-chord pivot to brick center
    % assumes chord direction is +x here based on your BrickDimensions usage
    xCOM = chord_i / 4;

    % ----------------------------
    % Solid properties
    % ----------------------------
    set_param(blkSolid, 'BrickDimensions', ...
        "[" + (chord_i) + ", L_t , " + (SectionWidth_Out) + "]");
 
    set_param(blkSolid, 'BasedOnType', 'Mass');
    set_param(blkSolid, 'MassUnits', 'kg');
    set_param(blkSolid, 'Mass', '0.01');

    set_param(blkSolid, 'GraphicDiffuseColor', ...
        "[" + (0.3 + 0.7*i/WingSecNum_Out) + " 0.4 0.8]");

    % ----------------------------
    % COM transform:
    % moves from quarter-chord pivot to solid center
    % ----------------------------
    set_param(blkCOMXfm, ...
        'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', ...
        "[" + num2str(xCOM) + " 0 " + num2str(zPivot) + "]", ...
        'RotationMethod', 'None');

    % ----------------------------
    % Revolute joint settings
    % ----------------------------
    set_param(blkJoint, 'MotionActuationMode', 'InputMotion');
    set_param(blkJoint, 'TorqueActuationMode', 'ComputedTorque');

    % ----------------------------
    % Gain settings
    % ----------------------------
    pitch_gain_i = pitch_gain(i);
    set_param(blkGain,  'Gain', num2str(pitch_gain(i)));
    set_param(blkGain1, 'Gain', num2str(pitch_gain(i)));
    set_param(blkGain2, 'Gain', num2str(pitch_gain(i)));

    % connect block to ref frame
    add_line(sprintf('%s/%s', model, OutboardLinkages), ...
        'Outboard_ref/LConn2', segName  + "/LConn1", 'autorouting', 'on');

    set_param(templateBlk, 'Commented', 'on');   % comment out

end

%% adds scope & connects a bunch of lins together
Wing_line_connect_automation;

%% Save
save_system(char(model));
disp("Wing creation complete.");

% run modification script
WingModify;