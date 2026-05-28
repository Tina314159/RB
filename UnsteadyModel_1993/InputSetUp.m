%% Input
noise_freq_cutoff = 20*ang_freq; %in rad/s (for low pass filter on h_ddot)

%% crank pos
crankPosTarget = -29; %deg
%crankPosTarget = -90; %deg (Fabio's ver)

%% assign input  (speed vs torque)
crankJoint = model + "/Crank/crankJoint";
delete_connected_lines(model+"/Crank/speedInput");
delete_connected_lines(model+"/Crank/torqueInput");
if (Speed_as_input == 1)
    set_param(crankJoint, 'MotionActuationMode', 'InputMotion');
    set_param(crankJoint, 'TorqueActuationMode', 'ComputedTorque');
    add_line(model+"/Crank", "crankJoint/LConn2", "speedInput/Rconn1", ...
        'autorouting', 'on');
    set_param(model + "/Solver Configuration", ...
    'UseLocalSolver', 'off');  
    dampingOfJoints = 0;
    ResetJointProperty;
elseif (Speed_as_input == -1)
    set_param(crankJoint, 'MotionActuationMode', 'ComputedMotion');
    set_param(crankJoint, 'TorqueActuationMode', 'InputTorque');
    add_line(model+"/Crank", "crankJoint/LConn2", "torqueInput/Rconn1", ...
        'autorouting', 'on');
    set_param(model + "/Solver Configuration", ...
    'UseLocalSolver', 'on');  
    dampingOfJoints = 0.001;
    ResetJointProperty;
    %set_param(crankJoint, 'PositionTargetSpecify', 'on');
    %set_param(crankJoint, 'PositionTargetValue', 'crankPosTarget');
end    
 
%% Base Crank Speed Input
T = 1/freq;                % flapping cycle period (2Hz = 0.5s)
t_tot = numPeriods*T;

% time vector
numSampPerT = 1250; % from [0,T), [T,2T),...
numSample = numPeriods*numSampPerT;
del_t = t_tot/(numSample);
t_samp_1T = linspace(0,T,numSampPerT+1);      % [0,T]
t_samp_1T = t_samp_1T(1:end-1);               % [0,T)
t_samp = linspace(0,t_tot,numSample+1);       % time (s)
t_samp = t_samp(1:end-1);               

% Equation set up
w0 = @(t) ang_freq;      % example trajectory (rad/s) over T
 
% Create time series
w0_data = w0(t_samp);      % operating condition
w0_ts = timeseries(w0_data,t_samp);

% assign input
assignin('base','omega_input',w0_ts);

%% Pitching input 

% example trajectory for pitching input (in rad)
pitch = @(t) pitch_amp * sin(ang_freq * (t-pitch_pha*T)); %del_pitch 
%pitch = @(t) pitch_amp * sin(ang_freq * (t-0.0*T)); 
pitch_data = pitch(t_samp);
pitch_data = min(pitch_lim, max(-1*pitch_lim, pitch_data)); 
    %saturate the value so it's between +- pitch_lim
pitch_data = fft_harmonic_approx(pitch_data, 20, figureOn) + pitch_mean;
    % = mean pitch + del pitch
pitch_ts = timeseries(pitch_data, t_samp);

pitch_data_lookup = pitch_ts.Data(1:numSampPerT); % first period

assignin('base','pitch_tip_in',pitch_ts);

% Other pitch values
% pitch_mean_verify = trapz(t_samp, pitch_data) / T; % rad
pitch_gain = linspace(0.1, 1, WingSecNum_Out); % percentage of pitch applied 
%pitch_gain = linspace(0., 0, WingSecNum_Out); % percentage of pitch applied 

%% Base Crank torque input
% simout1 = sim(model);
% tau1 = simout1.logsout.get('torque_out').Values.Data;
% tau1_t = simout1.logsout.get('torque_out').Values.Time;
% tau1_ts = timeseries(tau1, tau1_t);
% tau1_ts = resample(tau1_ts, t_samp);
% torque_ts = tau1_ts;

T0 = @(t) 1.;  
T0_data = T0(t_samp);      % operating condition
T0_ts = timeseries(T0_data,t_samp);
assignin('base','torque_input',T0_ts);


