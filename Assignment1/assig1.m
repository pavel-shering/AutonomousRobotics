clear all; clc;
close all;
% robots sizing 
L = 0.5; % [m]
r = 1; % [m]
f = 10; % [Hz]

% kinematic model
dt = 0.001;% [s] timestep 
x0 = [0; 0; pi()] % [[m/s] [m/s] [rad/s]] intial state
u = [-1.5; 2; 1] % [[rad/s] [rad/s] [rad/s]] inputs
u = [2; 0; 0]

% noise model 

n = 10000; % number of samples
figure(1); clf; hold on;

for i = 1:n
%     v1 = - u(1)*cos(x0(3)) + j*(u(1)*sin(x0(3)))
%     v2 = u(2)*cos(1/3*pi()-x0(3)) + j*(u(2)*sin(pi()/3-x0(3)))
%     v3 = u(3)*cos(1/3*pi()+x0(3)) - j* (u(3)*sin(1/3*pi()+x0(3)))
%     
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

quiver(x_record(1,:),x_record(2,:),v_record(1,:),v_record(2,:));
quiver(x_record(1,:),x_record(2,:),sin(x_record(3,:)),cos(x_record(3,:)),'r*');
