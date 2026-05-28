
pitch_pha = 0.30;
pitch_lim  = deg2rad(42);     % pitch amplitude
pitch_mean = deg2rad(20);
Speed_as_input = 1; % 1 = speed input, -1 = torque input
InputSetUp;

%% parameter sweep settings
dip_amp_vals = linspace(1, 9, 5);        % 5 values from 1 to 9
T_dip_vals   = linspace(0.01, 0.32, 5);  % 5 values from 0.01 to 0.32
t_peak_vals  = linspace(0.01, 0.32, 5);  % 5 values from 0.01 to 0.32

dip_amp_vals = linspace(7.5, 9, 5);
T_dip_vals   = linspace(0.27, 0.32, 5);
t_peak_vals  = linspace(0.22, 0.26, 5);
%% initialize storage
buffer = zeros(5,5,5);

delYItmax = -10;
ite_i_max = -1;
ite_j_max = -1;
ite_k_max = -1;

best_dip_amp = 0;
best_T_dip   = 0;
best_t_peak  = 0;

%% nested sweep
for ite_i = 1:5
    for ite_j = 1:5
        for ite_k = 1:5
            
            % set current parameter values
            dip_amp = dip_amp_vals(ite_i);
            T_dip   = T_dip_vals(ite_j);
            t_peak  = t_peak_vals(ite_k);

            % update model inputs / script
            SpeedInput_Modify;

            disp('start sim at indices:')
            disp([ite_i, ite_j, ite_k])
            disp('current [dip_amp, T_dip, t_peak] = ')
            disp([dip_amp, T_dip, t_peak])

            % simulate
            simout1 = sim(model);

            %% process data
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

            delY_data = delYin_data + delYout_data;
            delY_cyc_avg = delY_data(end) - delY_data((numPeriods-1)*numSampPerT);

            %% store in 5x5x5 buffer
            buffer(ite_i, ite_j, ite_k) = delY_cyc_avg;
            disp(delY_cyc_avg)
            %% track max
            if delY_cyc_avg > delYItmax
                delYItmax = delY_cyc_avg;

                ite_i_max = ite_i;
                ite_j_max = ite_j;
                ite_k_max = ite_k;

                best_dip_amp = dip_amp;
                best_T_dip   = T_dip;
                best_t_peak  = t_peak;
            end

            %% optional early stop
            if delYItmax > (4.0*T)
                disp('yeah! requirement met')
                disp('best indices = ')
                disp([ite_i_max, ite_j_max, ite_k_max]) 
            end

        end
 
    end
 
end

%% display final results
disp('max del Y momentum average in 1 cycle:')
disp(delYItmax)

disp('best indices [ite_i, ite_j, ite_k]:')
disp([ite_i_max, ite_j_max, ite_k_max])

disp('best [dip_amp, T_dip, t_peak]:')
disp([best_dip_amp, best_T_dip, best_t_peak])

disp('For a requirement of half-bird = 400g (need 4Ns momentum in 1sec)')
disp('we have (del Y momentum in 1 cycle)/(4N*1T) = ')
disp(delYItmax/(4*T))