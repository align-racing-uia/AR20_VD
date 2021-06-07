%Upright center i forhold til wheel center i geometri.m sitt koordinat
%system
UpFtykkelse = 50
UCG = Wc_FH-[0,Uptykkelse/2,0]

%Upright center midt i hullet til upright basert på lokalt koordinat system
UC =[0, 0, 0]

%Distanse fra center til indre ytre plan
UCInd = [25,0,0]
UCYtr = [25,0,0]

%21 mm til feste
%punkt til styrearms feste
ST = [8,-61,34]

%Upright feste på bunnen
FNYH = [0,-5.72,-82]

%Menge shims 
sh = 5
%lengde på shims vektor
ShimsY = sind(10.5)*sh*3
Shimsz = -cos(10.5)*sh*3

%lengde fra shims til feste punkt
FesteY = cosd(10.5)*21
FesteZ = -sind(10.5)*21

%Plassering av shims vektor
SForan = [6, -(2.51+ShimsY+FesteY), 84.2+Shimsz+FesteZ]

