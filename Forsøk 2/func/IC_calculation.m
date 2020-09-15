%     Finner Instantsenter(IC) sin y og z verdi. Instant akse
%     retiningsvektor(IC_DV). Instant akse og yz-planets
%     krysningspunkt(P_IC_yzPlan)

%     Input er matrisene for øvre og nedre whisbone og posisjonen til sentrum av hjulet(WheelCenter).
%     eksempel W_FOXV og W_FNXV
function [IC_XV,IC_XH , IC_DV_XV,IC_DV_XH , P_IC_yzPlan_XV,P_IC_yzPlan_XH] = IC_calculation(W_XOXX , W_XNXX , WheelCenter)

% gir nytt navn til punktene. Brukes kun i scriptet
P1 = W_XOXX(3,:);
P2 = W_XOXX(1,:);
P3 = W_XOXX(2,:);
P4 = W_XNXX(3,:);
P5 = W_XNXX(1,:);
P6 = W_XNXX(2,:);

% Finner Normalen for planet definert av 3 wishbone punkter
%onorm er øvre wishbone nnorm er nedre
onorm = NormFrom3P(P1 , P2 , P3);
nnorm = NormFrom3P(P4 , P5  , P6);

% finner krysnningspunktet mellom de to planene 
% bruker normalen til øvre og nedre wishbone og ytre punktet til wishbonene
[P_IC_yzPlan_XV , IC_DV_XV , c] = plane_intersect(onorm,P1,  nnorm,P4);

IC_DV_XH=IC_DV_XV .* [1 -1 1];
P_IC_yzPlan_XH = P_IC_yzPlan_XV .* [1 -1 1];

%for å finne hva Instant center(IC) brukes normalen til yz-planet(yzNorm)
% og x-verdien til Wheelcenter
yzNorm = [1 0 0];
V0 = [WheelCenter(1) 0 0];

%ut ifra Instant akse blir to punkter brukt til å finne trefingspunktet til
%instantaksen og et yz-plan plasert ved wheelCenter. P_end trenger kun å
%være så langt at V0 ligger mellom P_end og P_IC_yzPlan i x reting 
P_end = 4000 * IC_DV_XV + P_IC_yzPlan_XV;
[I , check] = plane_line_intersect(yzNorm , V0 , P_IC_yzPlan_XV , P_end);
IC_XV = I(2:3);
IC_XV = [WheelCenter(1) IC_XV(1) IC_XV(2)];
IC_XH=IC_XV .*[1 -1 1];


end