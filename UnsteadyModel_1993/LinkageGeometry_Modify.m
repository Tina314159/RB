%% linkage geometry modification 
% updates linkage length
% updates all the wing segments

%% Linkage info
% everthing is defined w.r.t L_D (crank length)
%L_D = 1.5/100;          % Crank length (m) 
%L_D = 3.7/100;           % updated ****************
%LinkageGeometrySetUp; % default Festo geometry

%% Discrete Wing info (Geometry)
% number of sections 
WingSecNum_In = 6;
WingSecNum_Out = 7;

%% mod start --------------------------------------------------------------
% tempo modification that will be removed later
%L_tip = (19.334/1.5)*L_D;

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
%% mod end ----------------------------------------------------------------

% Section width (m)
SectionWidth_In = L_Inboard/WingSecNum_In;
SectionWidth_Out = L_outboard/WingSecNum_Out;

% data from Avian_Wings.pdf (normalized by b/2 = semi-span)
% all in m
InboardCord = 0.35*L_D/0.037;     % cord length of inboard arm (~const)
meanCord = 0.33*L_D/0.037;        % can be used for simplifcation
                                  % scales by the factor b/c it's taken 
                                  %       for L_D = 3.7cm
                                  
% chord distribution for outboard: InboardCord -> 0.4*InboardCord
CordOut_root = InboardCord;
CordOut_tip  = 0.4 * InboardCord;
CordOut = linspace(CordOut_root, CordOut_tip, WingSecNum_Out);

% span & A & AR
b = 2 * (L_Inboard + L_outboard); % total wing span (m)
A = 2*(0.8*0.4);        % approximated using SegullWingPlanform (m^2)
A = A/(.037^2)*(L_D^2); % scales by the factor b/c A taken for L_D = 3.7cm
AR = b^2/A;  % AR = Aspect ratio = b/width = b^2/(width*b)=b^2/A
disp('Total Wing length (span) (m):')
disp(b)
disp('Total Wing area (2 wing) (m^2):')
disp(A)
disp('Aspect ratio (b^2/A):')
disp(AR)

% linkage motion
h_max = (0.3767/0.015)*L_D; 
                      % max plunging displacement at linkage tip 
                      % (tip of L_tip in m)
disp('Max plunging displacement (h_max in m):')
disp(h_max) 

% Aerodynamics (unit-less)
k = ang_freq * meanCord/ (2*U); 
                      % may want to update so it vary for each section ****************
k_inboard = k * InboardCord/meanCord;
k_outboard = k * CordOut/meanCord;

U_t = freq*h_max/U;   % Strohual Number 
                      % = max of (vertical average speed) / U
                      % want 0.2 < U_t < 0.4
disp('Reduced Frequency (k = wc/2U):')
disp(k)
disp('Strohual Number (U_t = f*h_max/U):')
disp(U_t)

% Modified Theodorsen function
C_1 = 0.5*AR/(2.32+AR);
C_2 = 0.181+0.772/AR;
F_k = @(k)   1 - C_1 *(k.^2)./(k.^2 +C_2^2);
G_k = @(k)   -C_1*C_2*k./(k.^2 +C_2^2);
C_k = @(k)   F_k(k) + 1i*G_k(k);
C_k_Jones = @(k)   AR*C_k(k)/(2+AR);
disp('Theodorsen Fcn mean val (C_k_Jones):')
disp(C_k_Jones(k))

%% recreates wing segments
WingCreationDeletion;