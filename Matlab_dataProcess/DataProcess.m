%Test Comment 
%% Plot data from load cell and myRio
clear;
clc;
clf;
close all;
% lengthen BTI or find better encoder. 
%% Load data and assign names
pdfName = 'leaf_60rpm_40IP_40deg.pdf';
LC_File = "LC_60rpm_40IP_30deg_maxWindv1.mat";
mR_File = "Leaf_60rpm_40IP_30deg.mat";
LC_data = load(LC_File);
mR_data = load(mR_File);

%% Load load cell data

% Extract variables 
t  = LC_data.t;

Fx = LC_data.Fx;
Fy =-1 * LC_data.Fy;
Fz = LC_data.Fz;

Tx = LC_data.Tx;
Ty = LC_data.Ty;
Tz = LC_data.Tz;

%% optional data re-zero
% Optional re-zeroing of load cell data
% Fx = Fx - mean(Fx(1:300));
% Fy = Fy - mean(Fy(1:300));
% Fz = Fz - mean(Fz(1:300));
% Tx = Tx - mean(Tx(1:300));
% Ty = Ty - mean(Ty(1:300));
% Tz = Tz - mean(Tz(1:300));

%% load myRio Data


% Extract variables

% Parameters
BTI            = mR_data.BTI;
targetvel      = mR_data.targetvel;
incrementvel   = mR_data.incrementvel;
incrementtime  = mR_data.incrementtime;
pitchoff       = mR_data.pitchoff;
pitchamp       = mR_data.pitchamp;

% Transient data
qrev_trans = mR_data.qrev_trans;
wrpm_trans = mR_data.wrpm_trans;
Va_trans   = -1 * mR_data.Va_trans;
Amp_trans  = mR_data.Amp_trans;
kp_trans   = mR_data.kp_trans;
ki_trans   = mR_data.ki_trans;

% Steady-state data
qrev_ss = mR_data.qrev_ss;
wrpm_ss = mR_data.wrpm_ss;
Va_ss   = -1 * mR_data.Va_ss;
Amp_ss  = mR_data.Amp_ss;
kp_ss   = mR_data.kp_ss;
ki_ss   = mR_data.ki_ss;

%% filter data
% Filter parameters
Fs = 1 / BTI;     % sampling frequency (Hz)
Fc = 1;          % cutoff frequency (Hz)
N  = 2;           % 2nd-order Butterworth
% Normalize cutoff frequency
Wn = Fc / (Fs/2);
% Design filter
[b, a] = butter(N, Wn, 'low');
% Apply filter

wrpm_trans_filt = filter(b, a, wrpm_trans);
wrpm_ss_filt = filter(b, a, wrpm_ss);
combined_w_filt = filter(b,a, [wrpm_trans.', wrpm_ss.']);
amp_trans_filt = filter(b, a, Amp_trans);
amp_ss_filt = filter(b, a, Amp_ss);


%% Create time vectors
t_trans = (0:length(wrpm_trans)-1) * BTI;
t_ss    = (0:length(wrpm_ss)-1) * BTI;

%% info
fig_info = figure('Color','w');
axis off

infoText = sprintf([ ...
    'Experiment Parameters\n\n' ...
    'Target Velocity      : %.2f RPM\n' ...
    'Velocity Increment   : %.2f RPM\n' ...
    'Increment Time       : %.2f s\n' ...
    'Pitch Offset         : %.3f\n' ...
    'Pitch Amplitude      : %.2f deg\n' ...
    'BTI                  : %.4f s\n' ...
    'KP                   : %.4f\n' ...
    'KI                   : %.4f\n\n' ...
    'Additional Comment: High wind, spreaded, no mR file...lost \n\n' ...
    'LC file              : %s \n' ...
    'mR file              : %s \n'], ...
    targetvel, incrementvel, incrementtime, ...
    pitchoff, pitchamp, BTI, kp_ss(1), ki_ss(1),LC_File,mR_File);

text(0.05,0.95,infoText, ...
    'Units','normalized', ...
    'VerticalAlignment','top', ...
    'FontName','Courier New', ...
    'FontSize',12, ...
    'Interpreter','none');

title('Experiment Summary');

%% Plot transient velocity
% figure;
% 
% plot(t_trans, wrpm_trans, 'LineWidth', 1.5,'Color','#FFA500');
% hold on
% plot(t_trans,  wrpm_trans_filt, 'LineWidth', 1.5,'Color','black')
% xlabel('Time (s)');
% ylabel('Velocity (RPM)');
% title('Transient Velocity Response');
% legend('raw data','filtered')
% grid on;

%% Plot steady-state velocity
% figure;
% 
% plot(t_ss, wrpm_ss, 'LineWidth', 1.5);
% hold on
% plot(t_ss, wrpm_ss_filt, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Velocity (RPM)');
% title('Steady-State Velocity');
% 
% grid on;

%% plot transient and steady state velocity on same plot
figure;

plot(t_trans, wrpm_trans, 'LineWidth', 1.5,'Color','#FFA500');
hold on;
plot(t_ss + t_trans(end), wrpm_ss, 'LineWidth', 1.5,'Color','red');
plot([t_trans, t_ss + t_trans(end)], combined_w_filt,'LineWidth', 1.5,'Color','black');

xlabel('Time (s)');
ylabel('Velocity (RPM)');
title('Velocity vs Time');
legend('Trasnient data', 'Steady State data', 'Filtered');
grid on;


%% Plot Controller Voltage and Motor Current
fig_elec = figure;
tiledlayout(2,1);

% Controller Output Voltage
nexttile;
plot(t_trans, Va_trans, 'LineWidth', 1.5);
hold on;
plot(t_ss + t_trans(end) + 5, Va_ss, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Controller Output Voltage (V)');
title('Controller Output Voltage');

legend('Transient', 'Steady-State');
grid on;

% Motor Current
nexttile;
plot(t_trans, Amp_trans, 'LineWidth', 1.5);
hold on;
plot(t_trans, amp_trans_filt, 'LineWidth', 1.5);
plot(t_ss + t_trans(end) + 5, Amp_ss, 'LineWidth', 1.5);
plot(t_ss + t_trans(end) + 5, amp_ss_filt, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Motor Current (A)');
title('Motor Current');

legend('Transient', 'Trans filtered', 'Steady', 'Steady filtered');
grid on;

%% Plot wing position
% fig_pos = figure;
% 
% plot(t_trans, qrev_trans, 'LineWidth', 1.5);
% hold on;
% plot(t_ss + t_trans(end), qrev_ss, 'LineWidth', 1.5);
% 
% xlabel('Time (s)');
% ylabel('Wing Position (rev)');
% title('Wing Position');
% 
% legend('Transient', 'Steady-State');
% 
% grid on;


%% Plot controller gains
% figure;
% 
% plot(t_trans, kp_trans, 'LineWidth', 1.5);
% hold on;
% plot(t_trans, ki_trans, 'LineWidth', 1.5);
% 
% xlabel('Time (s)');
% ylabel('Gain Value');
% title('Controller Gains During Transient');
% 
% legend('Kp', 'Ki');
% 
% grid on;

%% Display experiment info
fprintf('Target Velocity      : %.2f RPM\n', targetvel);
fprintf('Velocity Increment   : %.2f RPM\n', incrementvel);
fprintf('Increment Time       : %.2f s\n', incrementtime);
fprintf('Pitch Offset         : %.3f\n', pitchoff);
fprintf('Pitch Amplitude      : %.2f deg\n', pitchamp);
fprintf('BTI                  : %.4f s\n', BTI);
fprintf('KP                   : %.4f \n', kp_ss(1));
fprintf('KI                   : %.4f \n', ki_ss(1));


%% Plot Forces
fig_F = figure;
tiledlayout(5,1);

% Full time range
fig_F_ax1 = nexttile([2 1]);
plot(t, Fx, 'LineWidth', 1.5);
hold on;
plot(t, Fy, 'LineWidth', 1.5);
plot(t, Fz, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Force (N)');
title('Load Cell Forces - Full Range');
legend('Fx', 'Fy', 'Fz');
grid on;

% Zoomed time range
fig_F_ax2 = nexttile([2 1]);
plot(t, Fx, 'LineWidth', 1.5);
hold on;
plot(t, Fy, 'LineWidth', 1.5);
plot(t, Fz, 'LineWidth', 1.5);
%plot([t_trans (t_ss + t_trans(end))],[qrev_trans;qrev_ss ], 'LineWidth', 1.5,'Color','#DFFF00')

xlabel('Time (s)');
ylabel('Force (N)');
title('Load Cell Forces - Zoomed');
legend('Fx', 'Fy', 'Fz');
grid on;

% Position
fig_F_ax3 = nexttile;

plot([t_trans (t_ss + t_trans(end))], [qrev_trans; qrev_ss],'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (rev)');
title('Wing Position');
ylim([0 1]);
grid on;

% Apply zoom window
timeWindow = input('Enter the time window for zoomed-in plot (e.g., [start end]): ');
xlim(fig_F_ax2, timeWindow);
xlim(fig_F_ax3, timeWindow);

%% plot y momentum
% Integrate Fy over time
Fy_int = cumtrapz(t, Fy);

fig_Fy = figure;

plot(t, Fy, 'LineWidth', 1.5);
hold on;
% Plot integrated Fy
plot(t, Fy_int, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Force / Impulse (N)/(Ns)');
title('Load Cell Forces (y)');

legend('Fy', '\Delta y momentum');
grid on;

%% plot x momentum
% Integrate Fy over time
Fx_int = cumtrapz(t, Fx);

fig_Fx = figure;

plot(t, Fx, 'LineWidth', 1.5);
hold on;
% Plot integrated Fy
plot(t, Fx_int, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Force / Impulse (N)/(Ns)');
title('Load Cell Forces (x)');

legend('Fx', '\Delta x momentum');
grid on;

%% plot z momentum
% Integrate Fy over time
Fz_int = cumtrapz(t, Fz);

fig_Fz = figure;

plot(t, Fz, 'LineWidth', 1.5);
hold on;
% Plot integrated Fy
plot(t, Fz_int, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Force / Impulse (N)/(Ns)');
title('Load Cell Forces (z)');

legend('Fz', '\Delta z momentum');
grid on;

%% Plot Torques

fig_T = figure;
tiledlayout(2,1);

% Full time range
fig_T_ax1 = nexttile;
plot(t, Tx, 'LineWidth', 1.5);
hold on;
plot(t, Ty, 'LineWidth', 1.5);
plot(t, Tz, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Load Cell Torques - Full Range');
legend('Tx','Ty','Tz');
grid on;

% Zoomed time range
fig_T_ax2 = nexttile;
plot(t, Tx, 'LineWidth', 1.5);
hold on;
plot(t, Ty, 'LineWidth', 1.5);
plot(t, Tz, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Load Cell Torques - Zoomed');
legend('Tx','Ty','Tz');
grid on;

% Set different x-axis limits
timeWindow = input('Enter the time window for zoomed in plot (e.g., [start end]): ');
xlim(fig_T_ax2,timeWindow);    
%% save data to pdf

DataSavePDF