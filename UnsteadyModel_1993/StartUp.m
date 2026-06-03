%% Will always be run at the start of the project
clc
clear all

% note: everything was built using Matlab R2025b (different version might
% have different names for the blocks & ports)


%% Define key parameters 
U = 0.1;                % mean stream velocity (m/s)
freq = 3;                % flapping frequency (Hz)
ang_freq = freq*2*pi;    % flapping frequency (rad/s)
%AR = 5.0;               % Aspect Ratio

alpha_0 = deg2rad(0.5);  % Zero-lift-line angle (determined by airfoil)
Cd_cf = 1.5;             % cross flow drag coefficient
eta_s = 0.90;            % leading edge suction efficiency (guessing)
xi = 0.0;                % for stall behavior
alpha_stall_static = deg2rad(13); % Static stall angle (radians)

% dynamic viscosity of air (used for Re = rho*V*c/mu)
mu = 1.81e-5; % at room temp (Pa*s)
 
% recall for a Asin(wt) wave, max derivative = Aw, 
% which is limited by servo max speed. A = V/w
maxV_servo = 60/0.032;                         % deg/s
pitch_amp  = (deg2rad(maxV_servo))/ang_freq;   % Pitch def sine max amplitude (rad)
pitch_lim  = deg2rad(1);                      % pitch amplitude  
pitch_mean = deg2rad(1);                      % Mean pitch angle (radians)
pitch_pha = 0.30;                              % pitch phase shift (decimal percent)
% pitch_a mean value assumed to be zero. not added to equations

numPeriods = 4;            % number periods for time series & plots

%% diplay
figureOn = 0; % 0 = no figure shown, 1 = all figure shown

%% Linkage info
% everthing is defined w.r.t L_D (crank length)
%L_D = 1.5/100;          % Crank length (m) 
L_D = 3.7/100;           % updated ****************
%LinkageGeometrySetUp;    % default Festo geometry
LinkageGeometrySetUp_imp; % default Festo geometry with slight modification %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LinkageGeometry_Fabio_ver;

%% Discrete Wing info (Geometry)
% number of sections 
WingSecNum_In = 6;
WingSecNum_Out = 7;

% Section width (m)
SectionWidth_In = (L_Inboard/WingSecNum_In)*0.6; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SectionWidth_Out = L_outboard/WingSecNum_Out;

% data from Avian_Wings.pdf (normalized by b/2 = semi-span)
% all in m
InboardCord = 0.40;     % cord length of inboard arm (~const)
meanCord = 0.35;         % can be used for simplifcation

% chord distribution for outboard: InboardCord -> 0.4*InboardCord
CordOut_root = InboardCord;
CordOut_tip  = 0.4 * InboardCord;
CordOut = linspace(CordOut_root, CordOut_tip, WingSecNum_Out);

% span & A & AR
b = 2 * (L_Inboard + L_outboard); % total wing span (m)
A = 2*(0.8*0.4);        % approximated using SegullWingPlanform (m^2)
A = A/(.037^2)*(L_D^2); % scales by the factor b/c A taken for L_D = 3.7cm
AR = b^2/A;  % AR = Aspect ratio = b/width = b^2/(width*b)=b^2/A
disp('Total Wing length (span) (m):')
disp(b)
disp('Total Wing area (2 wing) (m^2):')
disp(A)
disp('Aspect ratio (b^2/A):')
disp(AR)

% linkage motion
h_max = (0.3767/0.015)*L_D;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      % max plunging displacement at linkage tip 
                      % (tip of L_tip in m)
disp('Max plunging displacement (h_max in m):')
disp(h_max) 


%% other parameters
% mass
M = 0.8;                % Total Bird Mass, kg
M_in = 0.205;            % Inboard arm mass (150g), kg
M_out = 0.133;           % Outboard arm mass (150g), kg
M_1wing = M_in+M_out;
M_in_i = M_in/WingSecNum_In;            % mass of each inboard segment
M_out_i = M_out*CordOut/sum(CordOut);   % mass of each outboard segment
M_half_body = 0.1;      % fuselage + battery etc = 0.2kg, so half = 0.1kg
% mass of servo
M_ser = 0.02; % 20 grams at wing tip

% Air property
rho = 1.225;          % Air Density (kg/m^3)

% Aerodynamics (unit-less)
k = ang_freq * meanCord/ (2*U); 
k_inboard = k * InboardCord/meanCord;
k_outboard = k * CordOut/meanCord;

U_t = freq*h_max/U;   % Strohual Number 
                      % = max of (vertical average speed) / U
                      % want 0.2 < U_t < 0.4
disp('Reduced Frequency (k = wc/2U):')
disp(k)
disp('Strohual Number (U_t = f*h_max/U):')
disp(U_t)

% Modified Theodorsen function
C_1 = 0.5*AR/(2.32+AR);
C_2 = 0.181+0.772/AR;
F_k = @(k)   1 - C_1 *(k.^2)./(k.^2 +C_2^2);
G_k = @(k)   -C_1*C_2*k./(k.^2 +C_2^2);
C_k = @(k)   F_k(k) + 1i*G_k(k);
C_k_Jones = @(k)   AR*C_k(k)/(2+AR);
disp('Theodorsen Fcn mean val (C_k_Jones):')
disp(C_k_Jones(k))

%% Open model
model = 'Linkage';
open_system(model)

%model2 = 'Linkage2';
%open_system(model2)

%% Input Set up (omega & pitch)
Speed_as_input = 1;     % 1 = speed input, -1 = torque input
pitch_lookup_input = 1; %  1 = look up table (more flexible)
                        % -1 = timeseires (faster simulation)                        
InputSetUp;

%% Inboard & Outboard angles
% crank = 0 deg in desmos when angle btw drive & ground = 0 deg
IB_OB_data = readmatrix('IB_OB_angle.csv');
x1_datas = IB_OB_data(:,1); % theta 1 in deg (when crank go from 0 to 360)
x2_datas = IB_OB_data(:,2); % theta 2 in deg

num_x_datas = length(x1_datas);
t_1period = linspace(0, T, num_x_datas);   % one full period
 
phase_data_lookup = linspace(0,2*pi,numSampPerT+1);
phase_data_lookup = phase_data_lookup(1:(end-1));

% repeats the mapping a few times
t_multi = linspace(0, numPeriods*T, numPeriods*num_x_datas);
x1_multi = repmat(x1_datas, numPeriods, 1);
x2_multi = repmat(x2_datas, numPeriods, 1);


%% input plot check
figure 
plot(t_multi, x1_multi);
hold on
plot(t_multi, x2_multi);
plot(t_samp,pitch_data * 180/pi);   %correct sign convention 
                                    % + angle of incedence/pitch = nose up
legend('Inboard angle (deg)','Outboard angle (deg)','Wing Tip Pitch (deg)')
title("Angle Values")
xlabel('Time (s)')
ylabel('Angle (deg)')

%% recreate wing  
WingCreationDeletion; % recreates wing each time, so it return to default 



