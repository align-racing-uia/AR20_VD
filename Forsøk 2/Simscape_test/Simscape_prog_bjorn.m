%Simscape test script. Run simscape progrematicly
clear
close
clc

%Definerer variabler
W = 0.01; %m
L = 0.2;

%Skrur av visualisering. NB: skrur av Permanent (ser det ut som) må skrus på igjen etterpå
set_param('mask_test','SimMechanicsOpenEditorOnUpdate','off');
%skjører simscape model (Simscape_prog) Simout er vinkel til revolut joint(Se to workspace block i model) 
simout = sim('mask_test');

%plotter tid og vinkel fra model
plot(simout.simout.Time, simout.simout.Data)

% Skrur på visualisering igjen.
set_param('mask_test','SimMechanicsOpenEditorOnUpdate','on');