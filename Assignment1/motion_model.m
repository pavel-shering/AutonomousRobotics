function [ mup ] = motion_model( state, input, dt )
% Outputs motion model from Mapping I slide 10

if ~exist('dt', 'var')
    dt = 0.1;
end

% mup = [state(1) + input(1)*cos(state(3))*dt;
%      state(2) + input(1)*sin(state(3))*dt;
%      state(3) + input(2)*dt];

v_x = (r*2/3) * (-input(1)*cos(state(3)) + input(2)*cos(pi()/3+state(3)) + input(3)*cos(pi()/3-state(3)));
v_y = (r*2/3) * (input(1)*sin(state(3))  - input(2)*sin(pi()/3+state(3)) + input(3)*sin(pi()/3-state(3)));
omega = r/(3*L) * (input(1)+u(2)+u(3));

mup = x0 + [ v_x * dt; v_y * dt; omega * dt];
end

