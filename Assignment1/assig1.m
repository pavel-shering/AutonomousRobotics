%% Motion Model
clear all; clc;
close all;
% robots sizing 
L = 0.3; % [m]
r = 0.25; % [m]
f = 10; % [Hz]

% kinematic model
x0 = [0; 0; 0]; % [[m/s] [m/s] [rad/s]] intial state
% u = [-1.5; 2; 1]; % [[rad/s] [rad/s] [rad/s]] inputs

% custom inputs
% u = [-1; 1; 0]; % [[rad/s] [rad/s] [rad/s]] to drive in a straight line
u = [1; -1; 2*pi/5/L*1.2]; % [[rad/s] [rad/s] [rad/s]] to drive in a spiral
% u = [1; -0.294745783575784; 0]; % 2m diamter circle inputs 

% Time
T = 15; % Duration
dt = 0.1;% [s] timestep (update rate)
tvec = 0:dt:T; % Time vector
n = length(tvec); % Number of timesteps

figure(1); clf; hold on;

x_record = zeros(3, n);
v_record = zeros(3, n);
y_record = zeros(3, n);

omega_variance = 0.1 / 180 * pi;

for i = 1:n
    % expanding spiral
%     u(3) = 2*(n - i) / n; %different spiral
    u(1) = u(1) + i * 0.001;% ever expanding spiral 
    u(2) = u(2) - i * 0.001;
    
    %measurement
    y = [x0(1); x0(2); x0(3)-99.7/180*pi] + [normrnd(0,0.5); normrnd(0,0.5); normrnd(0,10 / 180*pi)];

    % dynamics
    v_x = (r*2/3) * (-u(1)*cos(x0(3)) + u(2)*cos(pi/3-x0(3)) + u(3)*cos(pi/3+x0(3)));
    v_y = (r*2/3) * (u(1)*sin(x0(3)) + u(2)*sin(pi/3-x0(3)) - u(3)*sin(pi/3+x0(3)));
    omega = r/(3*L) * (u(1)+u(2)+u(3));
    
    x_record(:,i) = x0;
    v_record(:,i) = [v_x;
                     v_y;
                     omega];
    y_record(:,i) = y;
    
    %disturbance
    d = [normrnd(0,0.01); normrnd(0,0.01); normrnd(0,omega_variance)];
    
    x0 = x0 + [ v_x * dt; v_y * dt; omega * dt]% + d;
end

quiver(x_record(1,:),x_record(2,:),v_record(1,:),v_record(2,:));
quiver(x_record(1,:),x_record(2,:),sin(x_record(3,:)),cos(x_record(3,:)),'r*');
xlabel('xpos [m]');
ylabel('ypos [m]');
% quiver(y_record(1,:),y_record(2,:),zeros(n),zeros(n),'g*');
