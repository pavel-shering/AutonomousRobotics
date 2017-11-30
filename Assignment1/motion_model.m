function [ mup ] = motion_model( state, input, dt )
% Outputs motion model for the three wheeled robot

if ~exist('dt', 'var')
    dt = 0.1;
end
L = 0.3; % [m]
r = 0.25; % [m]

v_x = (r*2/3) * (-input(1)*cos(state(3)) + input(2)*cos(pi/3-state(3)) + input(3)*cos(pi/3+state(3)));
v_y = (r*2/3) * (input(1)*sin(state(3)) + input(2)*sin(pi/3-state(3)) - input(3)*sin(pi/3+state(3)));
omega = r/(3*L) * (input(1)+input(2)+input(3));


mup = state + [ v_x * dt; v_y * dt; omega * dt];
end