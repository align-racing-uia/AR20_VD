%% Roll center

% Dette scriptet gir ut Roll center(RC_X) i xyz kooridnater. 

% Scriptet bruker Punktene på venstresiden av bilen og antar att bilen er symetrisk om xz-planet.
% Scriptet regner ut for foran eller bak ut ifra hvilke innputtverider man gir. 

% Eksempel: [RC_F]=RC_calculation(W_FOXV , W_FNXV , Wc_FV , Tw_F , Td)
% Gir roll center forran

function [RC_X]=RC_calculation(W_XOXV , W_XNXV , Wc_XV , Tw_X , Td)

%Instant center venstre og høyere
[IC_XV,IC_XH , IC_DV_XV,IC_DV_XH , P_IC_yzPlan_XV,P_IC_yzPlan_XH] = IC_calculation(W_XOXV , W_XNXV , Wc_XV);

% Finner sentrum av kontaktflaten til dekket(C_of_contact_patch). Antar at C_of_contact_patch
% er plassert vinkelrett under Wheelcenter

C_of_contact_patch_XV=[Wc_XV(1) Tw_X/2 (Wc_XV(3)-(Td/2))];
C_of_contact_patch_XH=[Wc_XV(1) -Tw_X/2 (Wc_XV(3)-(Td/2))];

% Forenkler verdinavnene og setter opp så det kan gå inn i C_X = InterX(L1,L2)
% funksjonen. L1 og L2 er linjene spent mellom instant center og center of
% contact patch.
P_11 = IC_XV(2:3);
P_12 = C_of_contact_patch_XV(2:3);
P_21 = IC_XH(2:3);
P_22 = C_of_contact_patch_XH(2:3);
L1=[P_11(1) P_12(1) ; ...
    P_11(2) P_12(2)];
L2=[P_21(1) P_22(1) ; ...
    P_21(2) P_22(2)];
% Finner krysningspunktet
RC_X = InterX(L1,L2);

% Transponerer vektoren og legger til x-verdien så vektoren er i 3
% dimensjoner og radform.
RC_X=RC_X.';
RC_X=[Wc_XV(1) RC_X(1) RC_X(2)];
end