%% Update speed input trajectory

% round the corner of the curve without using fft. with max curvature
% given (max curvature = acceleration)

% controller automation
% how to discretize....

%% parameters
% dip_amp = 4;               % cosine modification amplitude
% T_dip = 0.3;              % width of one full dip (s)
% t_peak = 0.32;            % center time of dip within each cycle T (s)
 
%% constant speed input with custom dip 
% initialize
dip_func = @(t) dip_amp * ( cos((2*pi/T_dip) .* t) - 1);
dip_data = zeros(1,numSampPerT);

% logical mask where dip feature is active
dip_idx = (t_samp_1T <= T_dip);
 
% desired modification: A*(cos(wt) - 1)
dip_data(dip_idx) = dip_func(t_samp_1T(dip_idx));

% peak val shift
dip_peak_cur = round(0.5 * T_dip/del_t) +1; %matlab index start at 1
dip_peak_goal = round(t_peak/del_t) +1;
dip_data = circshift(dip_data, dip_peak_goal - dip_peak_cur);

% Total input trajectory
dip_data = ang_freq + dip_data;

% add missing amount so int(dip_data) = 2*pi
tempInt = trapz(t_samp_1T, dip_data);     %temporary integer storage
dip_data = dip_data + (2*pi - tempInt)/T;

% repeat mapping
% Repeat mapping for multiple periods
numPeriods = 4; % Number of periods to repeat
dip_data = repmat(dip_data, 1, numPeriods); 

% Create timeseries
w_ts = timeseries(dip_data, t_samp);

%% Assign to base workspace for Simulink / Simscape
assignin('base','omega_input',w_ts);

fprintf('\n================ Speed Modification Parameters ================\n');
fprintf('  T_dip (s): %.4f\n', T_dip);
fprintf('  Dip amplitude (rad/s): %.4f\n', dip_amp);
fprintf('  Dip peak time (s): %.4f\n', t_peak);
fprintf('  Max modified speed (rad/s): %.4f\n', max(dip_data));
fprintf('===============================================================\n\n');
%% Plot
if figureOn == 1
    figure;
    plot(t_samp, dip_data, 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Input Speed (rad/s)');
    title('Customized Periodic Speed Input');
    grid on;
end
%% sim
%sim(model)