%% Testrun

clear 
close
clc

addpath ./func
Geometri
%dynamikk script
[RC_F]=RC_calculation(W_FOXV , W_FNXV , Wc_FV , Tw_F , Td)
[RC_B]=RC_calculation(W_BOXV , W_BNXV , Wc_BV , Tw_B , Td)

%[AckI , AckY] = Ackermann(81, Wc_FH, SRH, SRV, UHV, Wc_FV); % Ackermann

Ilustrasjon

%Git test test
