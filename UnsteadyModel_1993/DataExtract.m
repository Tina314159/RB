%% Data Extract  
% Given:
%   - omega0(t) = specified trajectory (rad/s)
%   - T = flapping cycle period (2Hz = 0.5s)

%% Simulate
simout1 = sim(model);

%% extracting signals sim1
theta1 = simout1.logsout.get('theta').Values.Data;
theta1_t = simout1.logsout.get('theta').Values.Time;
theta1_ts = timeseries(theta1, theta1_t); 
theta1_ts = resample(theta1_ts, t_samp);

tau1 = simout1.logsout.get('torque_out').Values.Data;
tau1_t = simout1.logsout.get('torque_out').Values.Time;
tau1_ts = timeseries(tau1, tau1_t);
tau1_ts = resample(tau1_ts, t_samp);

omega1 = simout1.logsout.get('omega_out').Values.Data;
omega1_t = simout1.logsout.get('omega_out').Values.Time;
omega1_ts = timeseries(omega1, omega1_t);
omega1_ts = resample(omega1_ts, t_samp);

%% analyzing signals
tau1_data = squeeze(tau1_ts.Data);
w_data = squeeze(omega1_ts.Data);
theta1_data = squeeze(theta1_ts.Data);

figure
plot(t_samp,w_data, '.r');
hold on
plot(t_samp,tau1_data,'--b');
title("Torque vs. Rotational speed @" + freq + "Hz")
legend("\Omega Amplitude (rad/s)", "T Amplitude(Nm)")

%figure
%plot(t_samp,t)