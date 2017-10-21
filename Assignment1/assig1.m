clear all; clc;

% robots sizing 
L = 30; % [cm]
r = 25; % [cm]
f = 10; % [Hz]

% kinematic model
dt = 0.01;% [s] timestep 
x0 = [0; 0; 0] % [[m/s] [m/s] [rad/s]] intial state
u = [1; 1; 1] % [[rad/s] [rad/s] [rad/s]] inputs

% noise model 

n = 10; % number of samples
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
    x1 = x0 + [ (r/3)*dt*( -u(1)*cos(x0(3)) + u(2)*cos(pi()/3-x0(3)) + u(3)*cos(pi()/3+x0(3)) ) ;
                (r/3)*dt*( u(1)*sin(x0(3)) + u(2)*sin(pi()/3-x0(3)) - u(3)*sin(pi()/3+x0(3)) );
                (r/(3*L))*dt*(u(1)+u(2)+u(3))];
    x0 = x1        
end


