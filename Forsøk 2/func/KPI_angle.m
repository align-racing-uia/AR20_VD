%% KPI
%Bjørn Olav Karlsen

% Description
% function KPI_angle(Over,Under) takes the highest piont on the upright
% (Over) first, and the lower point on the upright (Under) and gives the
% KPI in degrees


function KPI = KPI_angle(FOYH , FNYH)

% Vektor between the two upright points
U = FOYH(2:3) - FNYH(2:3);
% Horisontal Vektor
V = [0,1];

UV_dot = dot(U,V);
UV_abs = sqrt( U(1)^2 + U(2)^2) * sqrt( V(1)^2 + V(2)^2);

%Cheking if KPI should be positive or negative
 if abs(FOYH(2)) < abs(FNYH(2))
     KPI = acosd(UV_dot/UV_abs);
 else
     KPI = -1 * acosd(UV_dot/UV_abs);
 end
end

