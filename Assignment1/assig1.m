clear all; clc;
close all;
% robots sizing 
L = 0.3; % [m]
r = 0.25; % [m]
f = 10; % [Hz]

% kinematic model
dt = 0.1;% [s] timestep (update rate)
x0 = [0; 0; 0]; % [[m/s] [m/s] [rad/s]] intial state
u = [-1.5; 2; 1]; % [[rad/s] [rad/s] [rad/s]] inputs
% u = [2; 2; 0];
%u = [2; -1.167612497392722; 0]; % 2m radius circle inputs 

% noise model 

n = 15 / dt; % number of samples for 15 second simulation
figure(1); clf; hold on;

x_record = zeros(3, n);
v_record = zeros(3, n);

omega_variance = 0.1 / 180 * pi();

for i = 1:n
    %u(3) = 2*(n - i) / n;

    % dynamics
    v_x = (r*2/3) * (-u(1)*cos(x0(3)) + u(2)*cos(pi()/3-x0(3)) + u(3)*cos(pi()/3+x0(3)));
    v_y = (r*2/3) * (u(1)*sin(x0(3)) + u(2)*sin(pi()/3-x0(3)) - u(3)*sin(pi()/3+x0(3)));
    omega = r/(3*L) * (u(1)+u(2)+u(3));
    
    rad = sqrt(v_x*v_x + v_y*v_y) / omega;
    d = rad * 2
    
    x_record(:,i) = x0;
    v_record(:,i) = [v_x;
                     v_y;
                     omega];
                 
    x1 = x0 + [ v_x * dt; v_y * dt; omega * dt] + [normrnd(0,0.01); normrnd(0,0.01); normrnd(0,omega_variance)];
    x0 = x1; 
end

% a = sqrt(v_record(1,:).^2 + v_record(2,:).^2)
samples_per_draw = 1;
trimmed_x_rec = x_record(:,1:samples_per_draw:end);
trimmed_v_rec = v_record(:,1:samples_per_draw:end);

quiver(trimmed_x_rec(1,:),trimmed_x_rec(2,:),trimmed_v_rec(1,:),trimmed_v_rec(2,:));
quiver(trimmed_x_rec(1,:),trimmed_x_rec(2,:),sin(trimmed_x_rec(3,:)),cos(trimmed_x_rec(3,:)),'r*');
