%% Plot data from load cell and myRio
clear;
clc;

%% Load load cell data
LC_data = load("loadcell_data.mat");

% Extract variables 
t  = LC_data.t;

Fx = LC_data.Fx;
Fy = LC_data.Fy;
Fz = LC_data.Fz;

Tx = LC_data.Tx;
Ty = LC_data.Ty;
Tz = LC_data.Tz;

%% load myRio Data

mR_data = load("Bird_Velocity_Control.mat");

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
Va_trans   = mR_data.Va_trans;
Amp_trans  = mR_data.Amp_trans;
kp_trans   = mR_data.kp_trans;
ki_trans   = mR_data.ki_trans;

% Steady-state data
qrev_ss = mR_data.qrev_ss;
wrpm_ss = mR_data.wrpm_ss;
Va_ss   = mR_data.Va_ss;
Amp_ss  = mR_data.Amp_ss;
kp_ss   = mR_data.kp_ss;
ki_ss   = mR_data.ki_ss;

%% Create time vectors
t_trans = (0:length(wrpm_trans)-1) * BTI;
t_ss    = (0:length(wrpm_ss)-1) * BTI;

%% Plot transient velocity
figure;

plot(t_trans, wrpm_trans, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Velocity (RPM)');
title('Transient Velocity Response');

grid on;

%% Plot steady-state velocity
figure;

plot(t_ss, wrpm_ss, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Velocity (RPM)');
title('Steady-State Velocity');

grid on;

%% Plot controller output voltage
figure;

plot(t_trans, Va_trans, 'LineWidth', 1.5);
hold on;
plot(t_ss + t_trans(end), Va_ss, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Controller Output Voltage (V)');
title('Controller Output Voltage');

legend('Transient', 'Steady-State');

grid on;

%% Plot motor current
figure;

plot(t_trans, Amp_trans, 'LineWidth', 1.5);
hold on;
plot(t_ss + t_trans(end), Amp_ss, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Motor Current (A)');
title('Motor Current');

legend('Transient', 'Steady-State');

grid on;

%% Plot wing position

figure;

plot(t_trans, qrev_trans, 'LineWidth', 1.5);
hold on;
plot(t_ss + t_trans(end), qrev_ss, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Wing Position (rev)');
title('Wing Position');

legend('Transient', 'Steady-State');

grid on;

%% Plot controller gains

figure;

plot(t_trans, kp_trans, 'LineWidth', 1.5);
hold on;
plot(t_trans, ki_trans, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Gain Value');
title('Controller Gains During Transient');

legend('Kp', 'Ki');

grid on;

%% Display experiment info

fprintf('Target Velocity      : %.2f RPM\n', targetvel);
fprintf('Velocity Increment   : %.2f RPM\n', incrementvel);
fprintf('Increment Time       : %.2f s\n', incrementtime);
fprintf('Pitch Offset         : %.3f\n', pitchoff);
fprintf('Pitch Amplitude      : %.2f deg\n', pitchamp);
fprintf('BTI                  : %.4f s\n', BTI);

%% Plot Forces
figure;

plot(t, Fx, 'LineWidth', 1.5);
hold on;
plot(t, Fy, 'LineWidth', 1.5);
plot(t, Fz, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Force (N)');
title('Load Cell Forces');

legend('Fx', 'Fy', 'Fz');
grid on;

%% Plot Torques
figure;

plot(t, Tx, 'LineWidth', 1.5);
hold on;
plot(t, Ty, 'LineWidth', 1.5);
plot(t, Tz, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Load Cell Torques');

legend('Tx', 'Ty', 'Tz');
grid on;