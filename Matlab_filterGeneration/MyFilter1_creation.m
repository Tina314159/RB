%%
clear all
clc

%% Butterworth low-pass filter
Fc = 10;         % cutoff frequency (Hz)      
Wc = 2*pi*Fc;    % rad/s       
N = 3;           % filter order (change as needed)       
T = 0.02;        % sampling time (BTI) in seconds (20ms)  

[b_f, a_f] = butter(N, Wc, 's');   % continuous-time Butterworth

Gf = tf(b_f, a_f);  %transfer function

%% Discretize filter
Gfp  = c2d(Gf, T, 'tustin'); % discrete controller in parallel form
Gft  = tf(Gfp);               % convert to transfer function

[b, a] = tfdata(Gft, 'v');
sos = tf2sos(b, a);

%% Export to C header
fid = fopen('myFilter1.h', 'w');
comment = ['2nd order Butterworth low-pass filter'];
sos2header(fid, sos, 'myFilter1', T, comment);
fclose(fid);