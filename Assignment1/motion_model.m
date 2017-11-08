function [ mup ] = motion_model( state, input, dt )
% Outputs motion model for the three wheeled robot

if ~exist('dt', 'var')
    dt = 0.1;
end
L = 0.3; % [m]
r = 0.25; % [m]

% % local frame velocities
% lv_x = (r/3) * (input(1)*cos(pi) + input(2)*cos(-pi/3) + input(3)*cos(pi/3));
% lv_y = (r/2) *  (input(2)*sin(-pi/3) + input(3)*sin(pi/3));
% 
% % global frame velocities
% v_x = lv_x * cos(state(3)) + lv_y * cos(state(3) + pi/2);
% v_y = lv_x * sin(state(3)) + lv_y * sin(state(3) + pi/2);
% 
% omega = r/(3*L) * (input(1)+input(2)+input(3));

v_x = (r*2/3) * (-input(1)*cos(state(3)) + input(2)*cos(pi/3-state(3)) + input(3)*cos(pi/3+state(3)));
v_y = (r*2/3) * (input(1)*sin(state(3)) + input(2)*sin(pi/3-state(3)) - input(3)*sin(pi/3+state(3)));
omega = r/(3*L) * (input(1)+input(2)+input(3));


mup = state + [ v_x * dt; v_y * dt; omega * dt];
end