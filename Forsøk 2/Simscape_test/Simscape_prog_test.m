%Simscape test script. Run simscape progrematicly
clear
close
clc

%Skrur av visualisering. NB: skrur av Permanent (ser det ut som) må skrus på igjen etterpå
set_param('Simscape_prog','SimMechanicsOpenEditorOnUpdate','off');

% Fro loop variabler
r1 = 1; %m
%L1 = 2;
i = 1;
for L1 = [1 2]

%skjører simscape model (Simscape_prog) Simout er vinkel til revolut joint(Se to workspace block i model) 
simtest(i) = sim('Simscape_prog');

i = i + 1;
end
%plotter tid og vinkel fra model
figure(1)
plot(simtest(1,1).joint1_pos.Time, simtest(1,1).joint1_pos.Data)
hold on
plot(simtest(1,2).joint1_pos.Time, simtest(1,2).joint1_pos.Data)

%Plotter velocity of pos for sett 1
figure(2)
plot(simtest(1,1).joint1_vel.Time, simtest(1,1).joint1_vel.Data)
hold on
plot(simtest(1,1).joint1_pos.Time, simtest(1,1).joint1_pos.Data)


% Skrur på visualisering igjen.
set_param('Simscape_prog','SimMechanicsOpenEditorOnUpdate','on');

