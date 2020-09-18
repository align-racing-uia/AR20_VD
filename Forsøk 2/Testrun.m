%% Testrun

clear 
close
clc

addpath ./func
Geometri
%dynamikk script
[RC_F]=RC_calculation(W_FOXV , W_FNXV , Wc_FV , Wb_F , Wd_F)
[RC_B]=RC_calculation(W_BOXV , W_BNXV , Wc_BV , Wb_B , Wd_B)

Ilustrasjon

%Git test test
