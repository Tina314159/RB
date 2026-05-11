clear
clc
num = 21;

m_tot = 0.3;
freq_vals =  linspace(0.1,10.1, num);
L12 = 0.037; 
L12_def = 0.037;
t14 = 56.8;
CL = 1;
CD = 0.1;
t14 = 40;

%out = repmat(struct(), length(t14_vals), 1);
lift = zeros(1, num);
tau  = zeros(1, num);
pow  = zeros(1, num);


for i = 1:num
    disp("iteration #:")
    disp(i)

    out_temp =RB_model(m_tot, freq_vals(i), L12, L12_def, t14, CL, CD);
    lift(i) = out_temp.lift_avg;
    tau(i)  = out_temp.tau_max;
    pow(i)  = out_temp.averagePower;
end
%%

figure
plot(freq_vals, lift, 'LineWidth', 1.5); % update
hold on
plot(freq_vals, tau,  'LineWidth', 1.5);
plot(freq_vals, pow,  'LineWidth', 1.5);

xlabel("Frequency (Hz)") % update
ylabel("Response")
title('Response vs. Drive Link Length');
legend("Lift (N)", "Torque (Nm)", "Power (W)")
grid on