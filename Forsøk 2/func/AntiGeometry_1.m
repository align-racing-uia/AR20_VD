function [ADV, ADH, ASV, ASH] = AntiGeometry( Wc_FV, Wc_BV, Wd_F, COG,...
FOFV, FOBV, FNFV, FNBV,...
FOFH, FOBH, FNFH, FNBH,...
BOFV, BOBV, BNFV, BNBV,...
BOFH, BOBH, BNFH, BNBH )
% Anti dive(braking forward)  and anti squat (accelerating

% center of gravity height
%COGH = COG(3);

sidebytte=[1  0  0; ...
           0 -1  0; ...
           0  0  1];%Bytter side, venstre til høyre osv.

% Length From wheel to wheel
L = Wc_FV(1) - Wc_BV(1);


%  Contact patch for all wheels
% Front left & Front right
% Wc_FV wheel center forran venstre osv...
Wcp_FV = Wc_FV - [0 0 Wd_F/2];
Wcp_FH = Wcp_FV * sidebytte;
% Back Left & Back right
Wcp_BV = Wc_BV - [0 0 Wd_F/2];
Wcp_BH = Wc_BV * sidebytte;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%Front Left Intersect
L1 = [FOFV(1) FOBV(1);FOFV(3) FOBV(3)];
L2 = [FNFV(1) FNBV(1);FNFV(3) FNBV(3)];
% Instantaneous center [Forran Venstre_Instantaneous Center]
FV_IC = InterX(L1,L2);

%Front Right Intersect
L1 = [FOFH(1) FOBH(1);FOFH(3) FOBH(3)];
L2 = [FNFH(1) FNBH(1);FNFH(3) FNBH(3)];
% Instantaneous center
FH_IC = InterX(L1,L2);

%Rear Left Intersect
L1 = [BOFV(1) BOBV(1);BOFV(3) BOBV(3)];
L2 = [BNFV(1) BNBV(1);BNFV(3) BNBV(3)];
% Instantaneous center
BV_IC = InterX(L1,L2);

%Rear Right Intersect
L1 = [BOFH(1) BOBH(1);BOFH(3) BOBH(3)];
L2 = [BNFH(1) BNBH(1);BNFH(3) BNBH(3)];
% Instantaneous center
BH_IC = InterX(L1,L2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for anti squat
IC_BVL = BV_IC(1) - Wc_BV(1);
IC_BVH = BV_IC(3) - Wc_BV(3);

IC_BHL = BH_IC(1) - Wc_BH(1);
IC_BHH = BH_IC(3) - Wc_BH(3);

% for anti dive (FVL = FORRAN Venstre Lengde)
IC_FVL = Wc_FV(1) - FV_IC(1);
IC_FVH = Wc_FV(3) - FV_IC(3);

IC_FHL = Wc_FH(1)- FH_IC(1);
IC_FHH = Wc_FH(3)- FH_IC(3);

% For anti dive we must multiply with the percentage breaking going to the
% front wheels. for racing cars this is usually 60/40% on front and back
% breaks respectively
break_rate_front = 0.6;

ASV  = (IC_BVH / IC_BVL)  / (CGH/L) *100; %anti squat Venstre
ASH  = (IC_BHH / IC_BHL)  / (CGH/L) *100;

% (L-CGL)given that the rear wheelbase/center has the x-coordinate 0
ADV  = break_rate_front* (IC_FVH / (L-IC_FVL)) * (CGH/ (L-CGL) ) *100; %anti dive Venstre
ADH  = break_rate_front* (IC_FHH / (L-IC_FHL)) * (CGH/ (L-CGL) ) *100;


end
