%% Running iterations (slow version)

%% key parameters
U = 0.1;                 % mean stream velocity (m/s)
freq = 2;                % flapping frequency (Hz)
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
maxV_servo = 60/0.032;                          % deg/s
pitch_amp  = (deg2rad(maxV_servo))/ang_freq;   % Pitch def sine max amplitude (rad)
pitch_lim  = deg2rad(1);                      % pitch amplitude  
pitch_mean = deg2rad(10);                      % Mean pitch angle (radians)
pitch_pha = 0.3;                              % pitch phase shift (decimal percent)
% pitch_a mean value assumed to be zero. not added to equations

%% simulation setup
numPeriods = 4;     % number periods for time series & plots
Speed_as_input = 1; % 1 = speed input, -1 = torque input
pitch_lookup_input = 1; %  1 = look up table (more flexible
set_param(model, 'StartTime', '0', 'StopTime', 'numPeriods*T');
InputSetUp;


%%
% for iter = 1:maxIterations
%     % simulate
%     pitch_mean = deg2rad(iter*10);
%     InputSetUp;
% 
%     dip_amp = 9;
%     T_dip   = 0.295;
%     t_peak  = 0.26;
%     % update model inputs / script
%     SpeedInput_Modify;
% 
%     disp('start sim #:')
%     disp(iter)
%     simout1 = sim(model);
% 
%     % process data (tau, omega, delYin, delYout,
%     tau1 = simout1.logsout.get('torque_out').Values.Data;
%     tau1_t = simout1.logsout.get('torque_out').Values.Time;
%     tau1_ts = timeseries(tau1, tau1_t);
%     tau1_ts = resample(tau1_ts, t_samp);
%     tau1_data = squeeze(tau1_ts.Data);
% 
%     omega1 = simout1.logsout.get('omega_out').Values.Data;
%     omega1_t = simout1.logsout.get('omega_out').Values.Time;
%     omega1_ts = timeseries(omega1, omega1_t);
%     omega1_ts = resample(omega1_ts, t_samp);
%     omega1_data = squeeze(omega1_ts.Data);
% 
%     power1_data = tau1_data.*omega1_data;
% 
%     delYin = simout1.logsout.get('delY_momentum_in').Values.Data;
%     delYin_t = simout1.logsout.get('delY_momentum_in').Values.Time;
%     delYin_ts = timeseries(delYin, delYin_t);
%     delYin_ts = resample(delYin_ts, t_samp);
%     delYin_data = squeeze(delYin_ts.Data);
% 
%     delYout = simout1.logsout.get('delY_momentum_out').Values.Data;
%     delYout_t = simout1.logsout.get('delY_momentum_out').Values.Time;
%     delYout_ts = timeseries(delYout, delYout_t);
%     delYout_ts = resample(delYout_ts, t_samp);
%     delYout_data = squeeze(delYout_ts.Data);
% 
%     delY_data = delYin_data+delYout_data;
%     delY_cyc_avg = (delY_data(end)-delY_data((numPeriods-1)*numSampPerT));
% 
%     delX_sum = simout1.logsout.get('delX_momentum').Values.Data;
%     delX_sum_t = simout1.logsout.get('delX_momentum').Values.Time;
%     delX_sum_ts = timeseries(delX_sum, delX_sum_t);
%     delX_sum_ts = resample(delX_sum_ts, t_samp);
%     delX_sum_data = squeeze(delX_sum_ts.Data);
%     delX_cyc_avg = (delX_sum_data(end)-delX_sum_data((numPeriods-1)*numSampPerT));
% 
% 
%     %store 
%     buffer(iter,1) = delY_cyc_avg;
%     if (delY_cyc_avg > delYItmax)
%         iteNum = iter; % Update iteration number
%         delYItmax = delY_cyc_avg; % Update maximum delY
%     end
% 
%     if (delYItmax > (4.0*T))
%         disp('yeah! at ite = ')
%         disp(iter) 
%     end
% end
% 
% % buffer2 = zeros(maxIterations,1);
% % for iter = 1:maxIterations
% %     % calculate buffer(iter)/(2.5*Ti)
% %     buffer2(iter,1) = buffer(iter,1)/(2.5*((iter/maxIterations * 6)^(-1))); 
% % end
% % buf2_max = max(buffer2);
% 
% disp('max del Y momentum average in 1 cycle:')
% disp(delYItmax)
% disp('at ite: ')
% disp(iteNum)
% disp('For a requirement of half-bird = 400g (need 4Ns momentum in 1sec)')
% disp('we have (del Y momentum in 1 cycle)/(4N*1T)=')
% disp(delYItmax/(4*T))


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Iteration setup
% initialize result storage

maxIterations = 10;

% pitch sweep
pitch_lim_list = 5*(1:maxIterations)-5;

% columns:
% 1 pitch_mean_deg
% 2 delX_cycle
% 3 delY_cycle
% 4 max_torque_amp
% 5 max_power
% 6 max_lift_force
summaryBuffer = zeros(maxIterations, 6);

delYItmax = -inf;  % key, want it to be >= 2.5 in 1s(250g, 2.5N, 2.5Ns=kg*m/s)
iteNum = -1;

for iter = 1:maxIterations
    % Update pitch value
    pitch_lim = deg2rad(pitch_lim_list(iter));
    InputSetUp;

    fprintf('\nstart sim #: %d\n', iter);
    fprintf('pitch lim = %.2f \n',pitch_lim);

    % Run simulation
    simout1 = sim(model);

    %% Read logged signals
    tau1_data = getLogResampled(simout1, 'torque_out', t_samp);
    omega1_data = getLogResampled(simout1, 'omega_out', t_samp);
    power1_data = getLogResampled(simout1, 'power', t_samp);

    L_in_data  = getLogResampled(simout1, 'L_tot_in',  t_samp);
    L_out_data = getLogResampled(simout1, 'L_tot_out', t_samp);
    T_in_data  = getLogResampled(simout1, 'T_tot_in',  t_samp);
    T_out_data = getLogResampled(simout1, 'T_tot_out', t_samp);

    delX_data = getLogResampled(simout1, 'delX_momentum', t_samp);

    delYin  = getLogResampled(simout1, 'delY_momentum_in',  t_samp);
    delYout = getLogResampled(simout1, 'delY_momentum_out', t_samp);
    delY_data = delYin + delYout;

    pitch_data = getLogResampled(simout1, 'pitch_m', t_samp);

    % Cycle averages & key values
    delY_cyc_avg = (delY_data(end)-delY_data((numPeriods-1)*numSampPerT));
    delX_cyc_avg = (delX_data(end)-delX_data((numPeriods-1)*numSampPerT));

    maxTorqueAmp = max(abs(tau1_data));
    maxPower     = max(abs(power1_data));

    L_total = L_in_data + L_out_data;
    maxLiftForce = max(abs(L_total));

    summaryBuffer(iter,:) = [
        pitch_pha, ...
        delX_cyc_avg, ...
        delY_cyc_avg, ...
        maxTorqueAmp, ...
        maxPower, ...
        maxLiftForce
    ];
%%
    if delY_cyc_avg > delYItmax
        iteNum = iter;
        delYItmax = delY_cyc_avg;
    end

    %% Print values for this iteration
    fprintf('Cycle delX momentum: %.6f Ns\n', delX_cyc_avg);
    fprintf('Cycle delY momentum: %.6f Ns\n', delY_cyc_avg);
    fprintf('Max torque amplitude: %.6f Nm\n', maxTorqueAmp);
    fprintf('Max power: %.6f W\n', maxPower);
    fprintf('Max lift force: %.6f N\n', maxLiftForce);

    %% Figure 1: forces and momentum
    figure('Name', sprintf('Iteration %d - Forces, Momentum, Motion & Power', iter));
    tiledlayout(4,1);

    nexttile;
    plot(t_samp, L_in_data,'Color','#20C1E3');
    hold on
    plot(t_samp, L_out_data,'Color','#205EE3');
    plot(t_samp, T_in_data,'Color','#D99A21');
    plot(t_samp, T_out_data,'Color','#D65236');
    title('Forces');
    xlabel('Time [s]');
    ylabel('Force [N]');
    legend('Inboard Lift','Outboard Lift','Inboard Thrust','Outboard Thrust')
    grid on;

    nexttile;
    plot(t_samp, delX_data,'Color','#D65236');
    hold on
    plot(t_samp, delY_data,'Color','#205EE3');
    title('\Delta Momentum');
    xlabel('Time [s]');
    ylabel('Momentum [Ns]');
    legend('x','y')
    grid on;

   
    nexttile;
    plot(t_samp, omega1_data);
    hold on;
    plot(t_samp, rad2deg(pitch_data)/10);
    %plot(t_samp, power1_data);
    plot(t_samp, tau1_data);
    legend('Crank Speed','10*Pitch','Torque');
    xlabel('Time [s]');
    ylabel('[rad/s],[deg],[W],[Nm]');
    grid on;

    nexttile;
    axis off;

    summaryStr = sprintf([ ...
    'Cycle Avg ΔX Momentum = %.4f Ns\n' ...
    'Cycle Avg ΔY Momentum = %.4f Ns\n' ...
    'Max Torque = %.4f Nm\n' ...
    'Max Power = %.4f W\n' ...
    'Max Lift = %.4f N'], ...
    delX_cyc_avg, ...
    delY_cyc_avg, ...
    maxTorqueAmp, ...
    maxPower, ...
    maxLiftForce);
    
    text(0.02,0.95,summaryStr, ...
        'Units','normalized', ...
        'VerticalAlignment','top', ...
        'FontSize',9, ...
        'FontName','Consolas', ...
        'Interpreter','none');
    
    title('Simulation Summary');
%%
end

%% Final printout
fprintf('\n================ Final Summary Buffer ================\n');
fprintf('Columns:\n');
fprintf('1 pitch_lim\n');
fprintf('2 delX_cycle_momentum\n');
fprintf('3 delY_cycle_momentum\n');
fprintf('4 max_torque_amp\n');
fprintf('5 max_power\n');
fprintf('6 max_lift_force\n\n');

disp(summaryBuffer);

fprintf('Max delY momentum average in 1 cycle:\n');
disp(delYItmax);

fprintf('At iteration:\n');
disp(iteNum);

fprintf('For requirement of half-bird = 400g, need 4 Ns momentum in 1 sec\n');
fprintf('We have delY / (4*T) = %.6f\n', delYItmax/(4*T));

%% Helper function
function data = getLogResampled(simout, sigName, t_query)
    rawData = simout.logsout.get(sigName).Values.Data;
    rawTime = simout.logsout.get(sigName).Values.Time;

    ts = timeseries(rawData, rawTime);
    ts = resample(ts, t_query);

    data = squeeze(ts.Data);
end