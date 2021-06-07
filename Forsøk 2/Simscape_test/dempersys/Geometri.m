%% Hjul relatert

% simulation veriables
% Lower and upper bound wheel rest
lower_bound = -50;
upper_bound = 100;

% Center of gravity height
CGH = 309;
%wheelbase
Wb=1552;

%track width
Tw_F = 1200; 
Tw_B = 1208;
%Wheel diameter (Felg)
Wd = 254;
%Tire Diameter
Td = 406; 

%Wheelcenter
Wc_FV = [Wb Tw_F/2 Td/2];
Wc_FH = [Wb -Tw_F/2 Td/2];

Wc_BV = [0 Tw_B/2 Td/2];
Wc_BH = [0 -Tw_B/2 Td/2];

%% Rammepunkter


sidebytte=[1  0  0; ...
           0 -1  0; ...
           0  0  1];%Bytter side, venstre til høyre osv.
%FV
FOFV = [1687 271.7 167];
FOBV = [1399 271.7 176];
FNFV = [1693.6 264 50.5];
FNBV = [1401 268 54.8];

%FH
FOFH = FOFV*sidebytte;
FOBH = FOBV*sidebytte;
FNFH = FNFV*sidebytte;
FNBH = FNBV*sidebytte;
%BV
BOFV = [510.9 229.6 210];
BOBV = [-8.82 259.5 210];
BNFV = [513 264.6 69];
BNBV = [-8.94 224.5 70];
%BH
BOFH = BOFV*sidebytte;
BOBH = BOBV*sidebytte;
BNFH = BNFV*sidebytte;
BNBH = BNBV*sidebytte;

%Lokalt koordinatsystem for Wishbone
 %FV
 defFOXV = FOBV;
 defFNXV = FNBV; 
 
 defFOFV = FOFV-defFOXV;
 defFOBV = FOBV-defFOXV;
 defFNFV = FNFV-defFNXV;
 defFNBV = FNBV-defFNXV;

 %FH
 defFOXH = defFOXV*sidebytte;
 defFNXH = defFNXV*sidebytte;
 
 defFOFH = FOFH-defFOXH;
 defFOBH = FOBH-defFOXH;
 defFNFH = FNFH-defFNXH;
 defFNBH = FNBH-defFNXH;
 
 %BV
 defBOXV = BOBV;
 defBNXV = BNBV; 
 
 defBOFV = BOFV-defBOXV;
 defBOBV = BOBV-defBOXV;
 defBNFV = BNFV-defBNXV;
 defBNBV = BNBV-defBNXV;

 %BH
 defBOXH = defBOXV*sidebytte;
 defBNXH = defBNXV*sidebytte;
 
 defBOFH = BOFH-defBOXH;
 defBOBH = BOBH-defBOXH;
 defBNFH = BNFH-defBNXH;
 defBNBH = BNBH-defBNXH;







%% Dempersystem-punkter



%Vi bruker mm
%foran

%rocker form (avstand fra rotasjonspunkt FR1H)
%FH
defFRXH = [1550 -246  60];
defFR1H = [   0    0   0]; %I forhold til FRXH
defFR2H = [   0  -84.34 -6.6]; %I forhold til FRXH
defFR3H = [   0 -115   4.66]; %I forhold til FRXH
defFR4H = [   0 -30.58   13.77]; %I forhold til FRXH

%FV
defFRXV = defFRXH * sidebytte;
defFR1V = defFR1H * sidebytte;
defFR2V = defFR2H * sidebytte;
defFR3V = defFR3H * sidebytte;
defFR4V = defFR4H * sidebytte;

%BH
defBRXH = [7.5 -305.45 325]; 
defBR1H = [0    0   0]; %i forhold til BRXH
defBR2H = [0    4.13  69.88]; %i forhold til BRXH
defBR3H = [0  -31.7 107.06]; %i forhold til BRXH
defBR4H = [0    46.33   12.76]; %i forhold til BRXH

%BV
defBRXV = defBRXH * sidebytte;
defBR1V = defBR1H * sidebytte;
defBR2V = defBR2H * sidebytte;
defBR3V = defBR3H * sidebytte;
defBR4V = defBR4H * sidebytte;

%Hele Rocker
defR_FRXH = [defFR1H ; defFR2H ; defFR3H ; defFR4H];
defR_FRXV = [defFR1V ; defFR2V ; defFR3V ; defFR4V];
defR_BRXH = [defBR1H ; defBR2H ; defBR3H ; defBR4H];
defR_BRXV = [defBR1V ; defBR2V ; defBR3V ; defBR4V];
%Rockerposisjon
%FH
FR1H = defFRXH + defFR1H;%Rocker festepunkt (rotasjonspunkt)
FR2H = defFRXH + defFR2H;%Rocker demper
FR3H = defFRXH + defFR3H; %Pull/pushrod
FR4H = defFRXH + defFR4H;%Rocker antirollbar
FDYH = FR2H;
FDIH = [1550 -255 237];%Innfestning demper
%FV
FR1V = FR1H * sidebytte;
FR2V = defFRXV + defFR2V;%Rocker demper
FR3V = defFRXV + defFR3V;
FR4V = defFRXV + defFR4V;%Rocker antirollbar
FDYV = FR2V;
FDIV = FDIH * sidebytte;
%BH
BR1H = [0 -250 380];%Rocker festepunkt (rotasjonspunkt)
BR2H = defBRXH + defBR2H;%Rocker demper
BR3H = defBRXH + defBR3H;
BR4H = defBRXH + defBR4H;%Rocker antirollbar
BDYH = BR2H;
BDIH = [7.5 -125 330];%Innfestning demper
%BV
BR1V = BR1H * sidebytte;
BR2V = defBRXV + defBR2V;%Rocker demper
BR3V = defBRXV + defBR3V;
BR4V = defBRXV + defBR4V;%Rocker antirollbar
BDYV = BR2V;
BDIV = BDIH * sidebytte;



%hoyere
FPYH = [1550-4.815 -550.805 289.96];%Ytre Push
FPIH = FR3H;%Inde Push
%venstre
FPIV = FR3H * sidebytte;
FPYV = [1550-4.815 550.805 289.96];%dette er bærekula i upright

%bak
%venstre
BPIV = BR3V;%[0,-250,102];%WHOOPS, HVA SKAL EGENTLIG SKJE HER?
BPYV = [0 550.805 289.96];

%hoyere
BPIH = BPIV*sidebytte;
BPYH = BPYV*sidebytte;

%hele dempesys
D_FXXH = [FPYH ; FPIH ; FR1H ; FR2H ; FR3H ; FR4H ; FDYH ; FDIH];
D_FXXV = [FPYV ; FPIV ; FR1V ; FR2V ; FR3V ; FR4V ; FDYV ; FDIV];
D_BXXH = [BPYH ; BPIH ; BR1H ; BR2H ; BR3H ; BR4H ; BDYH ; BDIH];
D_BXXV = [BPYV ; BPIV ; BR1V ; BR2V ; BR3V ; BR4V ; BDYV ; BDIV];

%initialvinkel til rocker i forhold til xz-plan
theta_RFH = atan(abs((FR2H(2) - FR1H(2))) / (abs(FR2H(3) - FR1H(3))));
theta_RFV = atan(abs((FR2V(2)-FR1V(2)))/(abs(FR2V(3)-FR1V(3))));
theta_RBH = atan(abs((BR2H(2)-BR1H(2)))/(abs(BR2H(3)-BR1H(3))));
theta_RBV = atan(abs((BR2V(2)-BR1V(2)))/(abs(BR2V(3)-BR1V(3))));

%% Uprightpunkter


%FH
FOYH = [1550-5.79  -538.53  220.45];
FNYH = [1550+5.72  -562.99  57];
 defFOYH = FOYH - defFOXH;
 defFNYH = FNYH - defFNXH;
 defFPYH = FPYH - defFOXH;

%FV
FOYV = FOYH * sidebytte;
FNYV = FNYH * sidebytte;
 defFOYV = FOYV - defFOXV;
 defFNYV = FNYV - defFNXV;
 defFPYV = FPYV - defFOXV;

%BH
BOYH = [0 -513.6 224.33];
BNYH = [0 -538.84 61];
 defBOYH = BOYH - defBOXH;
 defBNYH = BNYH - defBNXH;
 defBPYH = BPYH - defBOXH;
 
%Venstre
BOYV = BOYH * sidebytte;
BNYV = BNYH * sidebytte;
 defBOYV = BOYV - defBOXV;
 defBNYV = BNYV - defBNXV;
 defBPYV = BPYV - defBOXV;
%% Wishbone matriser

%FO
defW_FOXH = [defFOFH ; defFOBH ; defFOYH ; defFPYH];
defW_FOXV = [defFOFV ; defFOBV ; defFOYV ; defFPYV];
W_FOXH = [FOFH ; FOBH ; FOYH ; FPYH];
W_FOXV = [FOFV ; FOBV ; FOYV ; FPYV];

%FN
defW_FNXH = [defFNFH ; defFNBH ; defFNYH];
defW_FNXV = [defFNFV ; defFNBV ; defFNYV];
W_FNXH = [FNFH ; FNBH ; FNYH];
W_FNXV = [FNFV ; FNBV ; FNYV];
%BO
defW_BOXH = [defBOFH ; defBOBH ; defBOYH ; defBPYH];
defW_BOXV = [defBOFV ; defBOBV ; defBOYV ; defBPYV];
W_BOXH = [BOFH ; BOBH ; BOYH ; BPYH];
W_BOXV = [BOFV ; BOBV ; BOYV ; BPYV];
%BN
defW_BNXH = [defBNFH ; defBNBH ; defBNYH];
defW_BNXV = [defBNFV ; defBNBV ; defBNYV];
W_BNXH = [BNFH ; BNBH ; BNYH];
W_BNXV = [BNFV ; BNBV ; BNYV];

%statisk vinkel til foran/over/hoyre berearm i forhold til xy-plan
% theta_WO=atan(defFPYH(3)/defFPYH(2));
% theta_WN=atan(defFNYH(3)/defFNYH(2));

%% Styregeometri

%Stryresnekke (SR=Stearing Rack)
SRV=[1451.14  213 90.74];
SRH=[1451.14  -213 90.74];
%Upright handlebar
UHV=[1491.4  528  109.7];
UHH=[1491.4  -528  109.7];

%% lengder

%lengde på armer
  %FH
  l_FOFH = norm(FOYH - FOFH);
  l_FOBH = norm(FOYH - FOBH);
  l_FNFH = norm(FNYH - FNFH);
  l_FNBH = norm(FNYH - FNBH);
  %FV
  l_FOFV = norm(FOYV - FOFV);
  l_FOBV = norm(FOYV - FOBV);
  l_FNFV = norm(FNYV - FNFV);
  l_FNBV = norm(FNYV - FNBV);
  %BH
  l_BOFH = norm(BOYH - BOFH);
  l_BOBH = norm(BOYH - BOBH);
  l_BNFH = norm(BNYH - BNFH);
  l_BNBH = norm(BNYH - BNBH);
  %BV
  l_BOFV = norm(BOYV - BOFV);
  l_BOBV = norm(BOYV - BOBV);
  l_BNFV = norm(BNYV - BNFV);
  l_BNBV = norm(BNYV - BNBV);
%lengden til demper
%FH
l_FDXH = norm(FDYH - FDIH);
l_FR12H = norm(FR2H - FR1H);
%FV
l_FDXV = norm(FDYV - FDIV);
l_FR12V = norm(FR2V - FR1V);
%BH
l_BDXH = norm(BDYH - BDIH);
l_BR12H = norm(BR2H - BR1H);
%BV
l_BDXV = norm(BDYV - BDIV);
l_BR12V = norm(BR2V - BR1V);
%pushrod lengde
l_FPXH = norm(FPYH - FPIH);
l_FPXV = norm(FPYV - FPIV);
l_BPXH = norm(BPYH - BPIH);
l_BPXV = norm(BPYV - BPIV);

%lengde mellom upright punkter
l_FXYH = norm(FOYH - FNYH);
l_FXYV = norm(FOYV - FNYV);
l_BXYH = norm(BOYH - BNYH);
l_BXYV = norm(BOYV - BNYV);