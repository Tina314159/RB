%% Discrete Wing Modification (updated for WingSeg_In_i / WingSeg_Out_i)
% This script only modifies existing segments.
% It does NOT create or delete them.

%% some important block paths
InboardLinkages  = "Parallelogram";
OutboardLinkages = "Wing Tip";

inboardLevel  = model + "/" + InboardLinkages;
outboardLevel = model + "/" + OutboardLinkages;

%% control model wing modification
% inboardLevel  = modelC + "/Plant/" + InboardLinkages;
% outboardLevel = modelC + "/Plant/" + OutboardLinkages;

%% Modify inboard wing segments

for i = 1:WingSecNum_In

    segName = "WingSeg_In_" + num2str(i);
    segBlk  = inboardLevel + "/" + segName;

    % internal block paths inside each copied subsystem
    blkSolid  = segBlk + "/WingSolid";
    blkJoint  = segBlk + "/Wing_Joint";
    blkCOMXfm = segBlk + "/Xfm";
    blkGain   = segBlk + "/pitchGain";
    blkGain1  = segBlk + "/pitchGain1";
    blkGain2  = segBlk + "/pitchGain2";
    blkLeadingEdge = segBlk + "/1993dL&dT/LeadingEdge";
    blkF14         = segBlk + "/1993dL&dT/F1//4";
    blkF34         = segBlk + "/1993dL&dT/F3//4";


    % section geometry
    chord_i = InboardCord;

    % Solid dimensions
    set_param(blkSolid, 'BrickDimensions', ...
        "[InboardCord ,L_t , SectionWidth_In ]");
 
    % Mass properties
    set_param(char(blkSolid), 'BasedOnType', 'Mass');
    set_param(char(blkSolid), 'MassUnits', 'kg');
 
    set_param(char(blkSolid), 'Mass', 'M_in_i');   % if M_in_i is intended as workspace variable/expression

    % Color
    set_param(char(blkSolid), 'GraphicDiffuseColor', ...
        "[" + num2str(i/WingSecNum_In) + " 0.5 0.5]");

    % COM / frame transform
    zPivot = (2*i - 1) * SectionWidth_In / 2 + L_F*0.3; % spanwise location 
    xCOM = chord_i * 0.7/ 4; % chordwise location %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set_param(blkCOMXfm, ...
        'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', ...
        "[" + num2str(xCOM) + " 0 " + num2str(zPivot) + "]", ...
        'RotationMethod', 'None');

    % Joint settings
    set_param(char(blkJoint), 'MotionActuationMode', 'InputMotion');
    set_param(char(blkJoint), 'TorqueActuationMode', 'ComputedTorque');

    % Gain settings
    % inboard: all zero
    set_param(char(blkGain),  'Gain', '0');
    set_param(char(blkGain1), 'Gain', '0');
    set_param(char(blkGain2), 'Gain', '0');

    % Chord Frame transformation
    set_param(char(blkLeadingEdge), ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', "[" + num2str(-2*chord_i/4) + " 0 0]", ...
    'RotationMethod', 'None');

    set_param(char(blkF14), ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', "[" + num2str(-1*chord_i/4) + " 0 0]", ...
    'RotationMethod', 'None');

    set_param(char(blkF34), ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', "[" + num2str(chord_i/4) + " 0 0]", ...
    'RotationMethod', 'None');
end

%% Modify outboard wing segments

for i = 1:WingSecNum_Out

    segName = "WingSeg_Out_" + num2str(i);
    segBlk  = outboardLevel + "/" + segName;

    % internal block paths inside each copied subsystem
    blkSolid  = segBlk + "/WingSolid";
    blkJoint  = segBlk + "/Wing_Joint";
    blkCOMXfm = segBlk + "/Xfm";
    blkGain   = segBlk + "/pitchGain";
    blkGain1  = segBlk + "/pitchGain1";
    blkGain2  = segBlk + "/pitchGain2";
    blkLeadingEdge = segBlk + "/1993dL&dT/LeadingEdge";
    blkF14         = segBlk + "/1993dL&dT/F1//4";
    blkF34         = segBlk + "/1993dL&dT/F3//4";

    % Section geometry
    chord_i = CordOut(i);

    % spanwise location of section quarter-chord pivot
    zPivot = (2*i - 1) * SectionWidth_Out / 2;

    % offset from quarter-chord pivot to solid center
    xCOM = chord_i * 0.7 / 4;

    % Solid properties
    set_param(char(blkSolid), 'BrickDimensions', ...
        "[" + num2str(chord_i) + ", L_t, SectionWidth_Out]");

    set_param(char(blkSolid), 'BasedOnType', 'Mass');
    set_param(char(blkSolid), 'MassUnits', 'kg');
    set_param(char(blkSolid), 'Mass', num2str(M_out_i(i)));

    set_param(char(blkSolid), 'GraphicDiffuseColor', ...
        "[" + num2str(0.3 + 0.7*i/WingSecNum_Out) + " 0.4 0.8]");

    % COM transform
    set_param(char(blkCOMXfm), ...
        'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', ...
        "[" + num2str(xCOM) + " 0 " + num2str(zPivot) + "]", ...
        'RotationMethod', 'None');

    % Revolute joint settings
    set_param(char(blkJoint), 'MotionActuationMode', 'InputMotion');
    set_param(char(blkJoint), 'TorqueActuationMode', 'ComputedTorque');

    % Gain settings
    set_param(char(blkGain),  'Gain', num2str(pitch_gain(i)));
    set_param(char(blkGain1), 'Gain', num2str(pitch_gain(i)));
    set_param(char(blkGain2), 'Gain', num2str(pitch_gain(i)));

   % Chord Frame transformation
   set_param(char(blkLeadingEdge), ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', "[" + num2str(-2*chord_i/4) + " 0 0]", ...
    'RotationMethod', 'None');

    set_param(char(blkF14), ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', "[" + num2str(-1*chord_i/4) + " 0 0]", ...
    'RotationMethod', 'None');

    set_param(char(blkF34), ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', "[" + num2str(chord_i/4) + " 0 0]", ...
    'RotationMethod', 'None');
end

%% Save
save_system(char(model));
disp("Discrete wing modification complete.");