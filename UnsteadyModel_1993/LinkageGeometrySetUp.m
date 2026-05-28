%% Linkage Geometry

% Crank system --------------------------------
%L_D = 1.5/100;          % Crank length (m)
%L_D = 3.7/100;            % updated ****************
L_G = (4.796/1.5)*L_D;  % ground length
L_C = (3.589/1.5)*L_D;  % coupler equivalent length
L_C1 = (2.445/1.5)*L_D; % Coupler long segment
L_C2 = (1.238/1.5)*L_D; % coupler short segment
L_F = (16.189/1.5)*L_D; % upper follower length
L_Fp = (4.051/1.5)*L_D; % CF joint to pivot distance
A_G = 56.801;           % angle between ground and horizontal axis
A_Fp = 10.448;          % angle between L_F & L_Fp (deg)
A_CC1 = 9.156;          % angle between L_C1 & L_C (deg)

% Parallelogram --------------------------------
L_Fl = (16.118/1.5)*L_D;% lower follower length
L_EG = (1.265/1.5)*L_D;

% wing tip
L_EF = (1.494/1.5)*L_D;
L_FH = (1.002/1.5)*L_D;
L_GH = (1.129/1.5)*L_D; 
L_tip = (19.334/1.5)*L_D;

% other parameter
L_t =0.5/100;            %  Member Thicknesses (m)

L_Inboard = L_Fl + L_GH;
L_outboard = L_tip+0.05;

% wing length display
disp('Inboard Arm length (joint C to H):')
disp(L_Inboard)
disp('Outboard Arm length (Tip bar length + 5cm flap):')
disp(L_outboard)
disp('Total Wing length (semispan) (m):')
disp(L_Inboard + L_outboard)