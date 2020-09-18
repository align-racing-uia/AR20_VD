function [castorang] = CastorAngle(FOYX,FNYX)
%CASTORANGLE Summary of this function goes here
%   Detailed explanation goes here
a = [FOYX(1)-FNYX(1) ,FOYX(3)-FNYX(3)]
b = [0, 1]

castorang = acosd((dot(a,b) / (norm(a)*norm(b))));

if a(1)<b(1)
    castorang = castorang.*-1;
elseif a(1)==b(1)
    castorang=0;
else
end
castorang = real(castorang);
end

