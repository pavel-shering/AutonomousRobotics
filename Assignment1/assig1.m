clear all; clc;
close all;
% robots sizing 
L = 0.3; % [m]
r = 0.25; % [m]
f = 10; % [Hz]

% kinematic model
dt = 0.001;% [s] timestep 
x0 = [0; 0; 0] % [[m/s] [m/s] [rad/s]] intial state
%u = [-1.5; 2; 1] % [[rad/s] [rad/s] [rad/s]] inputs
 u = [2; 2; 0]

% noise model 

n = 15 / dt; % number of samples for 15 second simulation
figure(1); clf; hold on;

for i = 1:n
    u(3) = 2*(n - i) / n;
%incorrect velocity equations, look at the bottom equations.
%     theta = i;
%     v1 = - u(1)*cos(x0(3)) + j*(u(1)*sin(x0(3)))
%     v2 = u(2)*cos(1/3*pi()-x0(3)) + j*(u(2)*sin(pi()/3-x0(3)))
%     v3 = u(3)*cos(1/3*pi()+x0(3)) - j* (u(3)*sin(1/3*pi()+x0(3)))
%     
%    u = [v1;v2;v3]/r;

    %     abs (v1)
%     abs (v2)
%     abs (v3)
%     
%     x0(3) = x0(3) + 0.1;
%     
%     plot([v1, v2, v3], '*')
    % dynamics
    x_record(:,i) = x0;
    v_record(:,i) = [(r*2/3)*( -u(1)*cos(x0(3)) + u(2)*cos(pi()/3-x0(3)) + u(3)*cos(pi()/3+x0(3)) );
                     (r*2/3)*( u(1)*sin(x0(3)) + u(2)*sin(pi()/3-x0(3)) - u(3)*sin(pi()/3+x0(3)) );
                     r/(3*L)*(u(1)+u(2)+u(3))];
                 
    x1 = x0 + [ (r*2/3)*( -u(1)*cos(x0(3)) + u(2)*cos(pi()/3-x0(3)) + u(3)*cos(pi()/3+x0(3)) )*dt;
                (r*2/3)*( u(1)*sin(x0(3)) + u(2)*sin(pi()/3-x0(3)) - u(3)*sin(pi()/3+x0(3)) )*dt;
                (r/(3*L))*(u(1)+u(2)+u(3))*dt];
    x0 = x1; 
end

% a = sqrt(v_record(1,:).^2 + v_record(2,:).^2)
samples_per_draw = 100;
trimmed_x_rec = x_record(:,1:samples_per_draw:end);
trimmed_v_rec = v_record(:,1:samples_per_draw:end);

quiver(trimmed_x_rec(1,:),trimmed_x_rec(2,:),trimmed_v_rec(1,:),trimmed_v_rec(2,:));
quiver(trimmed_x_rec(1,:),trimmed_x_rec(2,:),sin(trimmed_x_rec(3,:)),cos(trimmed_x_rec(3,:)),'r*');
