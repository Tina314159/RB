%% start up 3 - control files
modelC = "LinkageC";  % C stands for control file

%% parameters of system
SpeedControlYes = 1; %1 = speed control, -1 = power control
GearRatio = 1/2;

%Kp = 0.001;
%Ki = 0.01;

Ka = 10;        % current amplifier constant A/V 
Km = 73;        % motor constant (Nm/A)

TorqueLimit = 7.2; % in Nm (this sets voltage lim to torquelim/KaKm)

P_ref = 10;         % power reference
W_ref = 3*(2*pi);   % speed reference
%sim(modelC)

%% setup speed ref
Speed_as_input = 1;     % 1 = speed input, -1 = torque input
pitch_pha = 0.30;
pitch_lim  = deg2rad(1);     % pitch amplitude
pitch_mean = deg2rad(1);

InputSetUp;
%% speed modification
% dip_amp = 9;
% T_dip = 0.295;
% t_peak = 0.26;
% SpeedInput_Modify;
  
%% open model
open_system(modelC);
