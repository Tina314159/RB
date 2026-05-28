%% Control Data Signal 

%% Simulate control model
Speed_as_input = -1; % 1 = speed input, -1 = torque input
InputSetUp;

simout1 = sim(modelC);

%% extracting signals sim1 
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
 
figure
plot(t_samp,w_data, '.r');
hold on
plot(t_samp,tau1_data,'--b');
title("Torque vs. Rotational speed @" + freq + "Hz")
legend("\Omega Amplitude (rad/s)", "T Amplitude(Nm)")

%% Simulate linkage model
Speed_as_input = 1;
InputSetUp;
assignin('base','omega_input',omega1_ts);

simout2 = sim(model);

%% extract data
tau2 = simout2.logsout.get('torque_out').Values.Data;
tau2_t = simout2.logsout.get('torque_out').Values.Time;
tau2_ts = timeseries(tau2, tau2_t);
tau2_ts = resample(tau2_ts, t_samp);
tau2_data = squeeze(tau2_ts.Data);

%% compare result of 2 model
figure
plot(t_samp,tau1_data, '.r');
hold on
plot(t_samp,tau2_data,'--b');
title("Torque for different model @" + freq + "Hz")
legend("control model", "linkage model")

%% momentum data
% -------- simout1 --------
delYin1 = simout1.logsout.get('delY_momentum_in').Values.Data;
delYin1_t = simout1.logsout.get('delY_momentum_in').Values.Time;
delYin1_ts = timeseries(delYin1, delYin1_t);
delYin1_ts = resample(delYin1_ts, t_samp);
delYin1_data = squeeze(delYin1_ts.Data);

delYout1 = simout1.logsout.get('delY_momentum_out').Values.Data;
delYout1_t = simout1.logsout.get('delY_momentum_out').Values.Time;
delYout1_ts = timeseries(delYout1, delYout1_t);
delYout1_ts = resample(delYout1_ts, t_samp);
delYout1_data = squeeze(delYout1_ts.Data);

delY1_data = delYin1_data + delYout1_data;


% -------- simout2 --------
delYin2 = simout2.logsout.get('delY_momentum_in').Values.Data;
delYin2_t = simout2.logsout.get('delY_momentum_in').Values.Time;
delYin2_ts = timeseries(delYin2, delYin2_t);
delYin2_ts = resample(delYin2_ts, t_samp);
delYin2_data = squeeze(delYin2_ts.Data);

delYout2 = simout2.logsout.get('delY_momentum_out').Values.Data;
delYout2_t = simout2.logsout.get('delY_momentum_out').Values.Time;
delYout2_ts = timeseries(delYout2, delYout2_t);
delYout2_ts = resample(delYout2_ts, t_samp);
delYout2_data = squeeze(delYout2_ts.Data);

delY2_data = delYin2_data + delYout2_data;

figure
plot(t_samp,delY1_data, '.r');
hold on
plot(t_samp,delY2_data,'--b');
title("momentum data @" + freq + "Hz")
legend("control model", "linkage model")
