function [angle_FV,angle_FH] = staticToe(FOYH,FNYH,UHH, FOYV, FNYV, UHV ,BOYH,BNYH, BOYV, BNYV)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% convert to degrees
radToDeg = 180/pi;
% differense fra styrepinne

% to do:
% finne diferanse fra Emil's punkter
% og så finne vinkel

% Reff.Vector
v = [0,1]';

% Forran Venstre
FV = NormFrom3P(FOYV, FNYV, UHV);
% care about x and y
FV_unit = [FV(1), FV(2)]';
angle_FV = subspace(v,FV_unit) * radToDeg

% Forran Høyre
FH = NormFrom3P(FOYH, FNYH, UHH);
% care about x and y
FH_unit = [FH(1), FH(2)]';
angle_FH = subspace(v,FH_unit) * radToDeg


end

