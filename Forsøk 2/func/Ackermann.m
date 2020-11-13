function [AckermannIndre,AckermannYtre] = Ackermann(x)
%ACKERMANN Denne funskjonen regner ut ackermann for AR20/21
%   Bruker relevant geometri på bilen og renger ut ackermann angle i en
%   sving ved konstant fart. Regner med en "turn angle" fa x=0-81, som
%   beveger styresnekka  x*3 i positiv eller negativ retning. Ved hjelp av
%   dette og trigonemtri finner funksjonen ackermann angle for indre og
%   ytre hjul.

steeringangle = 0;
R=1;
while R<=2;
    radius=3; %radius of rack and pinion gear
    deltay = steeringangle*radius; %mengden styresnekka beveger seg
    
    lgd1 = Wc_FH-SRH; %lengden fra Stryesnekke til wheel center
    
    lgd2 = SRV-UHV; %lengde på upright
    
    lgdSS = SRV-SRH; %lengden på styresnekka
    lgdbil = norm(Wc_FH-Wc_FV); %lengde fra wheel centre til wheel centre
    
    D = Wc_FH(1)-SRV(1); %Lengde mellom Styresnekka og wheel centre i x-plan
    
    lgdHcos = (norm(lgdbil)-norm(lgdSS))/2-deltay; %lengde fra styresnekke til x plan fra wheel center
    lgdVcos = (norm(lgdbil)-norm(lgdSS))/2+deltay; %samme som lgdHcos men på venstre side
    
    ang1 = acosd((dot(lgd1,lgd2) / (norm(lgd1)*norm(lgd2)))); %vinker mellom upright og lgd1
    
    ang2 = atand(D/lgdHcos); %vinkel mellom lgdxcos og lgd1
    
    ang3 = ang1-ang2; %vinkel mellom lgdxcos og upright
    
    Lgd2Hcos = lgdHcos-norm(lgd2)*cosd(ang3); %lengder mellom steering arm og kingpin
    Lgd2Hsin = D-norm(lgd2)*sind(ang3);
    
    RaPHangle = 90-atand(Lgd2Hcos/Lgd2Hsin); %Rack and pinion angle
    
    ang2_2 = atand(D/lgdVcos); %venstre
    ang3_2 = ang1-ang2_2; %venstre
    
    Lgd2Vcos = lgdHcos-norm(lgd2)*cosd(ang3_2);
    Lgd2Vsin = D-norm(lgd2)*sind(ang3_2);
    
    RaPVangle = 90-atand(Lgd2Vcos/Lgd2Vsin);
    
    if steeringangle == 0;
        
        RaPzero = 90-atand(Lgd2Hcos/Lgd2Hsin);
        
        steeringangle = [x];
    else
        AckermannIndre = RaPHangle-RaPzero %ackermann - Rack and pinion angle
        
        AckermannYtre = RaPVangle-RaPzero %ackermann - rackand pinion angle
        
    end
    R=R+1;
end

