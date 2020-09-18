function [RC_X]=RC_calculation(W_XOXV , W_XNXV , Wc_XV , Wb_X , Wd_X)
%% RC_CALCULATION Roll center
[IC_XV,IC_XH , IC_DV_XV,IC_DV_XH , P_IC_yzPlan_XV,P_IC_yzPlan_XH] = IC_calculation(W_XOXV , W_XNXV , Wc_XV);
C_of_contact_patch_XV=[Wc_XV(1) Wb_X/2 (Wc_XV(3)-(Wd_X/2))];
C_of_contact_patch_XH=[Wc_XV(1) -Wb_X/2 (Wc_XV(3)-(Wd_X/2))];
P_11 = IC_XV;
P_12 = C_of_contact_patch_XV(2:3);
P_21 = IC_XH;
P_22 = C_of_contact_patch_XH(2:3);
L1=[P_11(1) P_12(1) ; ...
    P_11(2) P_12(2)];
L2=[P_21(1) P_22(1) ; ...
    P_21(2) P_22(2)];
RC_X = InterX(L1,L2);
RC_X=RC_X.';
end