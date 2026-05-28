%% Running iterations (geometry)

%% key parameters
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
maxV_servo = 60/0.032;                          % deg/s
pitch_amp  = (deg2rad(maxV_servo))/ang_freq;   % Pitch def sine max amplitude (rad)
% pitch_lim  = deg2rad(60);                      % pitch amplitude  
% pitch_mean = deg2rad(10);                      % Mean pitch angle (radians)
% pitch_pha = 0.15;                              % pitch phase shift (decimal percent)
% pitch_a mean value assumed to be zero. not added to equations

%% simulation setup
numPeriods = 4;     % number periods for time series & plots
Speed_as_input = 1; % 1 = speed input, -1 = torque input
pitch_lookup_input = 1; %  1 = look up table (more flexible
set_param(model, 'StartTime', '0', 'StopTime', 'numPeriods*T');
InputSetUp;

%% Iteration setup
maxIterations = 5;

% initialize result storage
powerItmin = 1;         
powerItmax = -1;
tauItmin = 1;
tauItmax = -1;
delYItmax = 0; % key, want it to be >= 4 in 1s(400g, 4N, 4Ns=kg*m/s)
iteNum = -1; 
iteNum2 = -1;
buffer = zeros(maxIterations,1);
buffer2 = zeros(maxIterations,1);
buffer3 = zeros(maxIterations,1);

%% optimized parameters for upcoming iteration
pitch_pha = 0.30;
pitch_lim  = deg2rad(42);     % pitch amplitude
pitch_mean = deg2rad(20);
InputSetUp;
%% apply speed modification (or not)
% dip_amp = 7.5;
% T_dip   = 0.295;
% t_peak  = 0.26;
% SpeedInput_Modify;

%% geometry values to test
% L_D = 3.7/100;
LinkageGeometrySetUp;

% A_G = 56.801;           % defaule angle between ground and horizontal axis
% L_EF = 0.0369;          % meter
% A_Fp = 10.4480; % deg
% A_Fp = 0; % deg
% 
% A_CC1 = 9.1560; % deg
% A_CC1_val = linspace(0, 9.1560, maxIterations);

% new
% A_G = 74;           % defaule angle between ground and horizontal axis
% L_EF = 0.045;          % meter
% A_Fp = 0; % deg
% A_CC1 = 6;
%LinkageGeometry_Fabio_ver;
%LinkageGeometry_Modify; % always applied
%% iteration
set_param(crankJoint, 'PositionTargetSpecify', 'on');
set_param('Linkage/Wing Tip/H', 'PositionTargetSpecify', 'on');
set_param('Linkage/Wing Tip/H', 'PositionTargetValue', '-90'); %originally -90. + = CW direction
crankPosTarget = -30; %origianlly -30

%%
L_D_val = linspace(1,7.4,maxIterations)*0.01;

for iter = 1:maxIterations
    L_D = L_D_val(iter);
    %LinkageGeometry_Fabio_ver;
    LinkageGeometry_Modify; % always applied
    disp('start sim #:')
    disp(iter)
    simout1 = sim(model);
    
    % process data (tau, omega, delYin, delYout,
    tau1 = simout1.logsout.get('torque_out').Values.Data;
    tau1_t = simout1.logsout.get('torque_out').Values.Time;
    tau1_ts = timeseries(tau1, tau1_t);
    tau1_ts = resample(tau1_ts, t_samp);
    tau1_data = squeeze(tau1_ts.Data);
    tau1_max_temp = max(tau1_data);
    tau1_min_temp = min(tau1_data);

    omega1 = simout1.logsout.get('omega_out').Values.Data;
    omega1_t = simout1.logsout.get('omega_out').Values.Time;
    omega1_ts = timeseries(omega1, omega1_t);
    omega1_ts = resample(omega1_ts, t_samp);
    omega1_data = squeeze(omega1_ts.Data);

    power1_data = tau1_data.*omega1_data;
    
    delYin = simout1.logsout.get('delY_momentum_in').Values.Data;
    delYin_t = simout1.logsout.get('delY_momentum_in').Values.Time;
    delYin_ts = timeseries(delYin, delYin_t);
    delYin_ts = resample(delYin_ts, t_samp);
    delYin_data = squeeze(delYin_ts.Data);

    delYout = simout1.logsout.get('delY_momentum_out').Values.Data;
    delYout_t = simout1.logsout.get('delY_momentum_out').Values.Time;
    delYout_ts = timeseries(delYout, delYout_t);
    delYout_ts = resample(delYout_ts, t_samp);
    delYout_data = squeeze(delYout_ts.Data);
    
    delY_data = delYin_data+delYout_data;
    delY_cyc_avg = (delY_data(end)-delY_data((numPeriods-1)*numSampPerT));

    delX_sum = simout1.logsout.get('delX_momentum').Values.Data;
    delX_sum_t = simout1.logsout.get('delX_momentum').Values.Time;
    delX_sum_ts = timeseries(delX_sum, delX_sum_t);
    delX_sum_ts = resample(delX_sum_ts, t_samp);
    delX_sum_data = squeeze(delX_sum_ts.Data);
    delX_cyc_avg = (delX_sum_data(end)-delX_sum_data((numPeriods-1)*numSampPerT));

    %store 
    buffer(iter,1) = delY_cyc_avg;
    buffer2(iter,1) = tau1_max_temp;
    buffer3(iter,1) = delX_cyc_avg;
    if (delY_cyc_avg > delYItmax)
        iteNum = iter; % Update iteration number
        delYItmax = delY_cyc_avg; % Update maximum delY
    end

    % if (delX_cyc_avg > delXItmax)
    %     iteNum = iter; % Update iteration number
    %     delYItmax = delY_cyc_avg; % Update maximum delY
    % end
    
    if (delYItmax > (4.0*T))
        disp('yeah! at ite = ')
        disp(iter) 
    end

    if (tau1_max_temp > tauItmax)
        iteNum2 = iter;
        tauItmax = tau1_max_temp; % Update maximum torque
    end
end

% buffer2 = zeros(maxIterations,1);
% for iter = 1:maxIterations
%     % calculate buffer(iter)/(2.5*Ti)
%     buffer2(iter,1) = buffer(iter,1)/(2.5*((iter/maxIterations * 6)^(-1))); 
% end
% buf2_max = max(buffer2);

disp('max del Y momentum average in 1 cycle:')
disp(delYItmax)
disp('at ite: ')
disp(iteNum)
% disp('For a requirement of half-bird = 400g (need 4Ns momentum in 1sec)')
% disp('we have (del Y momentum in 1 cycle)/(4N*1T)=')
% disp(delYItmax/(4*T))
disp('max torque:')
disp(tauItmax) 
disp('at ite: ')
disp(iteNum2)



%% rim
% Wing_length = [32.5, 30, 30, 32.5, 32.5, 30, 26, 23.5, 20];
% mounting =  [6, 5, 6, 7.5, 8.5, 7.5,6, 3.5, 1.5];
% (mounting./Wing_length).'*100 % mounting location
% 
% (Wing_length.')/31 % length scale factor