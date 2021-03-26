%% test values

r = 5e-3
L = 200e-3

wishbone_1 = [1 0.7 0.3]
wishbone_2 = [1 -0.7 0.3]
wishbone_norm = norm(wishbone_1)
wishbone_length = norm(wishbone_1)
w_unit_1 = wishbone_1/wishbone_length
w_unit_2 = wishbone_2/wishbone_length

w_1 = w_unit_1*L
w_2 = w_unit_2*L
w_L = norm(w_1)


%% angles
phi_1 = atand(w_1(2)/w_1(1))
phi_2 = atand(w_1(3)/w_1(2))

% transform vector cartesional
% w_rot_1 = w_L / 2*[cosd(phi_1) sind(phi_1) cosd(phi_2)] 


