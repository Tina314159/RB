%% Linkage Geometry Setup - Fabio's version

% Fabio's version
L_D = 3.019/100;
L_C = 7.885/100;  % coupler equivalent length
L_Fp = 14.288/100; % CF joint to pivot distance
L_G = 9.560/100;  % ground length
L_C1 = 4.893/100;
A_CC1 = -1.868;

%check:
A_G = 20;           % angle between ground and horizontal axis

A_Fp = 6.03;
L_F = 36.4/100; % upper follower length
L_Fl = 36.4/100;% lower follower length

L_EF = 3.6/100;
L_FH = 2/100;
L_GH = 1.6/100; 
L_tip = 55/100;
L_EG = 3/100;


% other parameter
L_t =0.5/100;            %  Member Thicknesses (m)


L_Inboard = L_Fl + L_GH;
L_outboard = L_tip+0.05;

% wing length display
disp('Fabio version:')

disp('Inboard Arm length (joint C to H):')
disp(L_Inboard)
disp('Outboard Arm length (Tip bar length + 5cm flap):')
disp(L_outboard)
disp('Total Wing length (semispan) (m):')
disp(L_Inboard + L_outboard)

