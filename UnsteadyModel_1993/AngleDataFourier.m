%% Simplified Wing Model Using Desmo Kinematic Model

%% data  
IB_OB_data = readmatrix('IB_OB_angle.csv');
x1_datas = IB_OB_data(:,1); %% theta 1
x1_datas = pitch_data;
%% FFT compute
num_x1_datas = length(x1_datas);
t_1period = linspace(0, T, num_x1_datas);   % one full period

X_coeff = fft(x1_datas)/num_x1_datas;   % normalize

%% Truncate to first N harmonic & check error
NumHarmonic = 20;  % start small

X_coeff_trunc = zeros(size(X_coeff));
X_coeff_trunc(1:NumHarmonic+1) = X_coeff(1:NumHarmonic+1);
X_coeff_trunc(end-NumHarmonic+1:end) = X_coeff(end-NumHarmonic+1:end);

x_approx = real(ifft(X_coeff_trunc*num_x1_datas));

error = max(abs(x1_datas - x_approx));
figure; plot(t_samp, x_approx); hold on; plot(t_samp,x1_datas)
%% Automated truncation & error check
for NumHarmonic = 1:floor(num_x1_datas/2)
    X_coeff_trunc = zeros(size(X_coeff));
    X_coeff_trunc(1:NumHarmonic+1) = X_coeff(1:NumHarmonic+1);
    X_coeff_trunc(end-NumHarmonic+1:end) = X_coeff(end-NumHarmonic+1:end);
    
    x_approx = real(ifft(X_coeff_trunc*num_x1_datas));
    
    error = max(abs(x1_datas - x_approx));
    
    if error < 0.09
        break
    end
end

disp(['Needed harmonics to get error <0.09: ', num2str(NumHarmonic)])

%% reconstruction from FFT 
omega0 = 2*pi/T;     % the fundamental frequency

X_a0 = real(X_coeff(1));     % DC term

%N = floor(M/2);      % max usable harmonics
NumHarmonic = 50;               % The actual number of harmonics used for approximation ------------------ 
X_a = zeros(1,NumHarmonic);
X_b = zeros(1,NumHarmonic);

for n = 1:NumHarmonic
    cn = X_coeff(n+1);     % positive frequency coefficient
    
    X_a(n) = 2*real(cn);
    X_b(n) = -2*imag(cn);
end

x_approx = X_a0*ones(size(t_1period));

for n = 1:NumHarmonic
    x_approx = x_approx ...
        + X_a(n)*cos(n*omega0*t_1period) ...
        + X_b(n)*sin(n*omega0*t_1period);
end

real(ifft(X_coeff*num_x1_datas));
 
%% Torque calculation (1 wing)
% parameters 
DragCoeff = 1.8;

% recall
% M_1wing = mass of 1 wing  
% assuming rectangular wing, uniform mass. (kg)
len = b/2;              % length of wing (m)
width = meanCord;       % width of wing (m)
Vmax = 2.54*freq/2;     % max wing tip speed at 2Hz = 2.54m/s. scale by f/2Hz
%A = len * width; 

B = 0.5 * rho * (Vmax^2) * A * DragCoeff;
J = 1/3 * M_1wing * (len^2);
disp('B (Ns/m)= ')
disp(B)
disp('J (kg*m^2)= ')
disp(J)
%J = 0.0331;     % max rotational inertia of the wing (kg*m^2)
%B = 0.2646;     % max damping (Ns/m)

q_rad = x_approx*pi/180;
dt = t_1period(2)-t_1period(1);

% differentiate
qdot = diff(q_rad)/dt;          % angular speed
t_qdot = t_1period(1:end-1);            % reduction in vector size from diff
qddot = diff(q_rad,2)/(dt^2);   % angular acceleration
t_qddot = t_1period(1:end-2); 

% torque & calculation
Torque = J*qddot + B*qdot(1:end-1);
Pmotor = (Torque .* qdot(1:end-1)); %% power = torque * anuglar speed
Tmotor = Pmotor/ ang_freq;                 %% assume no significant energy stored in coupler bar

%% plots

% plot of torque
figure
plot(t_qddot, Torque,'.r')
xlabel('Time [s]')
ylabel('Torque [Nm]')
title('Torque vs Time')
grid on

% plot of pos approximation vs actual
figure
plot(t_1period, q_rad,'.r');
hold on
plot(t_1period,x1_datas*pi/180,'.g');
xlabel('Time [s]')
ylabel('Angular pos [rad]')
title('IB Arm Angular pos approx vs. actual')
legend('Angular pos (FFT)','actual ang pos')
grid on

% plot of pos, vel, acc
figure
plot(t_1period, q_rad,'.r');
hold on
plot(t_qdot, qdot,'.b');
xlabel('Time [s]')
ylabel('Angular pos, vel [rad, rad/s]')

yyaxis right
ylabel('accelration [rad/s^2]')
plot(t_qddot, qddot,'.y');

title('IB Arm Angular pos,vel,acc vs Time')
legend('Angular pos (FFT)', 'Angular speed','Angular acceleration')
grid on

 
 