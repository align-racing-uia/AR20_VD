%Simscape test script. Run simscape progrematicly
% clear
% close
% clc
% Sjekker om Simscape modellen er Loaded, hvis 
Simscape_On_Test = slreportgen.utils.isModelLoaded('Simscape_prog');
if Simscape_On_Test == 0
    disp('Loading Simscape Model')
    load_system('Simscape_prog')
else
    disp('Simscape Model is already loaded')
end

%Skrur av visualisering. NB: skrur av Permanent (ser det ut som) må skrus på igjen etterpå
set_param('Simscape_prog','SimMechanicsOpenEditorOnUpdate','off');

% Posisjon til pendelvekt i 2D. Origo er festepunkt



%trenger r1 og L1

i = 1;
for test = 1
Pen_pos = [test,1];
L1 = hypot(Pen_pos(1), Pen_pos(2));
alpha = angle(Pen_pos(1)+j*Pen_pos(2));
r1 = 1; %m
%L1 = 2;
%skjører simscape model (Simscape_prog) Simout er vinkel til revolut joint(Se to workspace block i model) 
simtest(i) = sim('Simscape_prog');

disp('Simulation done')
i = i + 1;
end
% %plotter tid og vinkel fra model
% figure(1)
% plot(simtest(1,1).joint1_pos.Time, simtest(1,1).joint1_pos.Data)
% hold on
% plot(simtest(1,2).joint1_pos.Time, simtest(1,2).joint1_pos.Data)
% 
% %Plotter velocity of pos for sett 1
% figure(2)
% plot(simtest(1,1).joint1_vel.Time, simtest(1,1).joint1_vel.Data)
% hold on
% plot(simtest(1,1).joint1_pos.Time, simtest(1,1).joint1_pos.Data)


% Skrur på visualisering igjen.
set_param('Simscape_prog','SimMechanicsOpenEditorOnUpdate','on');

