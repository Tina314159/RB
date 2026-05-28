%% Simulation
% Note
%   - this is part of start up file

%% Simulation & Parameter Summary
%sim(model)

disp('================ Simulation Parameters ================')
% --- Flow / environment ---
fprintf('Flow Conditions:\n');
fprintf('  U (m/s): %.3f\n', U);

% --- Geometry ---
fprintf('\nWing Geometry:\n');
fprintf('  Aspect Ratio (AR): %.3f\n', AR);
fprintf('  Inboard segments: %d (width = %.4f m each)\n', ...
            WingSecNum_In, SectionWidth_In);
fprintf('  Outboard segments: %d (width = %.4f m each)\n', ...
            WingSecNum_Out, SectionWidth_Out);

% --- Airfoil / aero parameters ---
fprintf('\nAirfoil / Aerodynamics:\n');
fprintf('  alpha0 (deg): %.3f\n', rad2deg(alpha_0));
fprintf('  (Cd)_cf (crossflow drag): %.4f\n', Cd_cf);
fprintf('  eta_s (LE suction efficiency): %.3f\n', eta_s);
fprintf('  xi (stall parameter): %.3f\n', xi);

% --- Motion ---
fprintf('\nKinematics:\n');
fprintf('  Frequency (Hz): %.3f\n', freq);
fprintf('  Pitch amplitude (+-deg): %.3f\n', rad2deg(pitch_lim));
fprintf('  Pitch relative phase (decimal): %.3f\n', pitch_pha);
fprintf('  Pitch mean val (deg): %.3f\n', rad2deg(pitch_mean));
fprintf('  Tip flapping distance (m): %.3f\n', h_max);

% --- Stall ---
fprintf('\nStall Parameters:\n');
fprintf('  Static stall angle (deg): %.2f\n', rad2deg(alpha_stall_static));

% --- Servo ---
fprintf('\nServo Parameters:\n');
fprintf('  Max Servo Speed (deg/s): %.2f\n', maxV_servo);

disp('======================================================')