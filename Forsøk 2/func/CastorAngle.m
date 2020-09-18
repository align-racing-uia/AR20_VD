function [castorang] = CastorAngle(FOYX,FNYX)
%Function for finding the Castor angle. Input The two upright points.
% The Upper Point First and the Lower one Secound. Gives The castor angle
% in degrees

%   Defines the vektor between The two upright points in X-Z plane.
a = [FOYX(1)-FNYX(1) ,FOYX(3)-FNYX(3)]
b = [0, 1]

% Calculates the angle between a and b 
castorang = acosd((dot(a,b) / (norm(a)*norm(b))));

% Checks if the angle is positive or negative
if FOYX(1)<FNYX(1)
    castorang = castorang;
else
    castorang = castorang*-1;
end
end

