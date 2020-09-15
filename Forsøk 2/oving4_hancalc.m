
Fx = 38.4e3 %N
D = 150 %mm
r = 15 %mm
t = 20 %mm
Mb = 547500 %Nmm

sigma_nom1 = Fx/(t*(D-(2*r))) %MPA
k = 3 - 3.13*(2*r/D) - 3.66*(2*r/D)^2 - 1.53*(2*r/D)^3
k=2
sigma_A = sigma_nom1*k

sigma_nom2 = 12*Mb*r/(t*(D^3-(2*r)^3))
sigma_nom3 = 6*Mb*D/(t*(D^3-(2*r)^3))
sigma_tot = sigma_nom3+sigma_A