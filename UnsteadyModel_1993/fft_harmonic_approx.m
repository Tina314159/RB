function x_approx = fft_harmonic_approx(datas, NumHarmonic,figureOn)
% FFT_HARMONIC_APPROX
% Approximates a periodic signal using truncated FFT harmonics
%
% Inputs:
%   pitch_data   : signal values over one period
%   NumHarmonic  : number of harmonics to retain
%
% Output:
%   x_approx     : reconstructed signal using truncated harmonics

%% Prepare data
x1_datas = datas(:);   % ensure column vector
num_x1_datas = length(x1_datas);

T = 1;                                   % assume 1 period
t = linspace(0, T, num_x1_datas);

%% FFT compute
X_coeff = fft(x1_datas) / num_x1_datas;   % normalized FFT

%% Truncate harmonics
X_coeff_trunc = zeros(size(X_coeff));

% positive frequencies
X_coeff_trunc(1:NumHarmonic+1) = X_coeff(1:NumHarmonic+1);

% negative frequencies
X_coeff_trunc(end-NumHarmonic+1:end) = X_coeff(end-NumHarmonic+1:end);

%% Reconstruct signal
x_approx = real(ifft(X_coeff_trunc * num_x1_datas));

%% Error
error_val = max(abs(x1_datas - x_approx));
disp(['Max reconstruction error of fft: ', num2str(error_val)])

%% Plot
if (figureOn == 1)
    figure
    plot(t, x1_datas,'LineWidth',2)
    hold on
    plot(t, x_approx,'--','LineWidth',2)
    
    xlabel('Time (normalized period)')
    ylabel('Signal')
    legend('Original','Harmonic Approximation')
    title(['FFT Harmonic Approximation with ',num2str(NumHarmonic),' Harmonics'])
    grid on
end

end