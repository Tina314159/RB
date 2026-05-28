%% Linkage Geometry - the version implemented

% Crank system --------------------------------
%L_D = 1.5/100;          % Crank length (m)
%L_D = 3.7/100;            % updated ****************
L_G = (11.83/3.7)*L_D;  % ground length
L_C = (9.37/3.7)*L_D;  % coupler equivalent length
L_C1 = (5.91/3.7)*L_D; % Coupler long segment
L_C2 = (3.67/3.7)*L_D; % coupler short segment
L_F = (39.93/3.7)*L_D; % upper follower length
L_Fp = (9.99/3.7)*L_D; % CF joint to pivot distance
A_G = 55;           % angle between ground and horizontal axis
A_Fp = 9.156;          % angle between L_F & L_Fp (deg)
A_CC1 = 9.432;          % angle between L_C1 & L_C (deg)

% Parallelogram --------------------------------
L_Fl = L_F;% lower follower length
L_EG = L_C2;

% wing tip
L_EF = (4.468/3.7)*L_D;
L_FH = (2.842/3.7)*L_D;
L_GH = (3.046/3.7)*L_D; 
L_tip = (43/3.7)*L_D; 

% other parameter
L_t =0.5/100;            %  Member Thicknesses (m)

L_Inboard = L_Fl + L_GH;
L_outboard = L_tip+0.1;

% wing length display
disp('Inboard Arm length (joint C to H):')
disp(L_Inboard)
disp('Outboard Arm length (Tip bar length + 5cm flap):')
disp(L_outboard)
disp('Total Wing length (semispan) (m):')
disp(L_Inboard + L_outboard)