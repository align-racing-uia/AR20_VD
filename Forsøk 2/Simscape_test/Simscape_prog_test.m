%Simscape test script. Run simscape progrematicly
clear
close
clc

%Definerer variabler
r1 = 0.5; %m
L1 = 2;

%Skrur av visualisering. NB: skrur av Permanent (ser det ut som) må skrus på igjen etterpå
set_param('Simscape_prog','SimMechanicsOpenEditorOnUpdate','off');
%skjører simscape model (Simscape_prog) Simout er vinkel til revolut joint(Se to workspace block i model) 
simout = sim('Simscape_prog');

%plotter tid og vinkel fra model
plot(simout.simout.Time, simout.simout.Data)

% Skrur på visualisering igjen.
set_param('Simscape_prog','SimMechanicsOpenEditorOnUpdate','on');