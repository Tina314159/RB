function build_wing_template(mdl)

if ~bdIsLoaded(mdl)
    load_system(mdl);
end

ss = mdl + "/WingSectionTemplate";

% Create subsystem if it does not exist
if ~bdIsLoaded(mdl) || isempty(find_system(mdl,'SearchDepth',1,'Name','WingSectionTemplate'))
    add_block("simulink/Ports & Subsystems/Subsystem", ss, ...
        "Position", [100 100 350 300]);
end

open_system(ss);

% Example internal block names
blkSolid   = ss + "/SectionSolid";
blkSensor  = ss + "/VelSensor";
blkExtF    = ss + "/AeroForce";
blkPS2SLx  = ss + "/PS2SL_Vx";
blkPS2SLy  = ss + "/PS2SL_Vy";
blkPS2SLz  = ss + "/PS2SL_Vz";
blkAeroFcn = ss + "/AeroCalc";
blkSL2PSFx = ss + "/SL2PS_Fx";
blkSL2PSFy = ss + "/SL2PS_Fy";
blkSL2PSFz = ss + "/SL2PS_Fz";

% Add blocks (library paths may vary by release)
add_block("simulink/User-Defined Functions/MATLAB Function", blkAeroFcn, ...
    "Position", [560 120 700 200]);

% You will add the Simscape / Simscape Multibody blocks similarly.
% Example:
% add_block("sm_lib/Body Elements/Solid", blkSolid, ... );
% add_block("sm_lib/Sensors/Transform Sensor", blkSensor, ... );
% add_block("sm_lib/Forces and Torques/External Force and Torque", blkExtF, ... );

% Configure geometry after adding the Solid block
% set_param(blkSolid, "GeometryType", "Brick");
% set_param(blkSolid, "BrickDimensions", "[secChord secSpan secThickness]");

% Configure MATLAB Function contents
fcnCode = [
    "function [Fx,Fy,Fz,L,D,Vmag] = fcn(Vx,Vy,Vz,rho,S,CL,CD)" newline ...
    "V = [Vx; Vy; Vz];" newline ...
    "Vmag = norm(V);" newline ...
    "if Vmag < 1e-9" newline ...
    "    Fx = 0; Fy = 0; Fz = 0; L = 0; D = 0;" newline ...
    "    return;" newline ...
    "end" newline ...
    "q = 0.5*rho*Vmag^2;" newline ...
    "D = q*S*CD;" newline ...
    "L = q*S*CL;" newline ...
    "% Simplest placeholder directions:" newline ...
    "% drag opposite x-velocity, lift along +z" newline ...
    "u = V / Vmag;" newline ...
    "Fdrag = -D*u;" newline ...
    "Flift = [0;0;L];" newline ...
    "F = Fdrag + Flift;" newline ...
    "Fx = F(1); Fy = F(2); Fz = F(3);" newline ...
    "end"
];
set_param(blkAeroFcn, 'U', fcnCode);

% Then connect blocks with add_line(...)
% add_line(ss, ...)

save_system(mdl);
end