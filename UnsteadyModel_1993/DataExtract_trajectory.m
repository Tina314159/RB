%% Fabio's data format - modified

%% simulate
% assumed constant speed....else inboard & outboard angle don't match
simout1 = sim(model);

%% signal extraction 
% total vertical momentum change due to aerodynamic:
delY_momentum = simout1.logsout.get('delY_momentum').Values;
delY_momentum_ts = timeseries(delY_momentum.Data, delY_momentum.Time);
delY_momentum_ts = resample(delY_momentum_ts, t_samp);
delY_momentum_data = squeeze(delY_momentum_ts.Data);  

tau1 = simout1.logsout.get('torque_out').Values.Data;
tau1_t = simout1.logsout.get('torque_out').Values.Time;
tau1_ts = timeseries(tau1, tau1_t);
tau1_ts = resample(tau1_ts, t_samp);
tau1_data = squeeze(tau1_ts.Data);  

omega1 = simout1.logsout.get('omega_out').Values.Data;
omega1_t = simout1.logsout.get('omega_out').Values.Time;
omega1_ts = timeseries(omega1, omega1_t);
omega1_ts = resample(omega1_ts, t_samp);
omega1_data = squeeze(omega1_ts.Data);  

L_tot_in1 = simout1.logsout.get('L_tot_in').Values;
L_tot_in1_ts = timeseries(L_tot_in1.Data, L_tot_in1.Time);
L_tot_in1_ts = resample(L_tot_in1_ts, t_samp);
L_tot_in1_data = squeeze(L_tot_in1_ts.Data);

L_tot_out1 = simout1.logsout.get('L_tot_out').Values;
L_tot_out1_ts = timeseries(L_tot_out1.Data, L_tot_out1.Time);
L_tot_out1_ts = resample(L_tot_out1_ts, t_samp);
L_tot_out1_data = squeeze(L_tot_out1_ts.Data);

L_tot_data = L_tot_in1_data + L_tot_out1_data;
%% other info
% total vertical momentum change due to gravity:
grav_momentum = 9.81 * (M_1wing + M_half_body) * t_tot; 

% recall angle 1 & 2 are defined 
% t_multi = linspace(0, numPeriods*T, numPeriods*num_x_datas);
% x1_multi = repmat(x1_datas, numPeriods, 1);
% x2_multi = repmat(x2_datas, numPeriods, 1);
 
netF = L_tot_data - 9.81 * (M_1wing + M_half_body);  % aerodynamic vertical force - weight

%% trajectory integration
z = zeros(length(t_samp),1);   % vertical position
v = zeros(length(t_samp),1);   % vertical velocity
z(1) = 10;                   % initial height
v(1) = 0;                    % initial vertical speed
ToF = -1;                    % time of fall / ground hit

for i = 2:length(t_samp)
    dt = t_samp(i) - t_samp(i-1);
    v(i) = v(i-1) + (netF(i)/(M_1wing + M_half_body)) * dt;
    z(i) = z(i-1) + v(i) * dt;

    if z(i) < 0 && z(i-1) >= 0
        ToF = t_samp(i);
    end
end

 
%%  plotting 
figure;

subplot(3,1,1);
plot(t_samp, netF);
xlim([0, t_tot]);
title("Vertical Force vs Time");
xlabel("Time [s]");
ylabel("Net Force [N]");

subplot(3,1,2);
plot(t_samp, tau1_data, 'DisplayName', 'Torque');
hold on; 
yyaxis right
plot(t_multi, x1_multi, 'DisplayName', 'Angle of inboard');
plot(t_multi, x2_multi, 'DisplayName', 'Angle of outboard');
ylabel("Angle [deg]");
 
title("Torque / Angle vs Time");
xlim([0, t_tot]);
ylabel("Torque [N m]");
legend show;
hold off;

subplot(3,1,3);
plot(t_samp, z);
xlim([0,t_tot]);
title("Trajectory over Time");
ylabel("Height [m]");
xlabel("Time [s]");

%%  summary metrics 
netI = delY_momentum_data(end) - grav_momentum(end);
FactorNeedeForSuccess = netI / delY_momentum_data(end);

disp("Time of fall (if reached ground):");
disp(ToF);

disp("Net impulse after gravity:");
disp(netI);

disp("Factor = netI / Imp(end):");
disp(FactorNeedeForSuccess);