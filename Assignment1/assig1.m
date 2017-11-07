%% Motion Model
clear all; clc;
close all;
% robots sizing 
L = 0.3; % [m]
r = 0.25; % [m]
f = 10; % [Hz]

% kinematic model
x0 = [0; 0; 0]; % [[m/s] [m/s] [rad/s]] intial state
u = [-1.5; 2; 1]; % [[rad/s] [rad/s] [rad/s]] inputs

% custom inputs
% u = [-1; 1; 0]; % [[rad/s] [rad/s] [rad/s]] to drive in a straight line
% u = [-1; 1; 1]; % [[rad/s] [rad/s] [rad/s]] to drive in a spiral
% u = [2; -1.167612497392722; 0]; % 2m radius circle inputs 

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
    % u(3) = 2*(n - i) / n; different spiral
    % u(1) = u(1) - i * 0.01; % ever expanding spiral 
    % u(2) = u(2) + i * 0.01;
    
    %measurement
    y = [x0(1); x0(2); 99.7/180*pi - x0(3)] + [normrnd(0,0.5); normrnd(0,0.5); normrnd(0,10 / 180*pi)];

    % dynamics
    % local frame velocities                          
    lv_x = (r/3) * (u(1)*cos(pi) + u(2)*cos(-pi/3) + u(3)*cos(pi/3));
    lv_y = (r/2) *  (u(2)*sin(-pi/3) + u(3)*sin(pi/3));

    % globbal fram velocities 
    v_x = lv_x * cos(x0(3)) + lv_y * cos(x0(3) + pi/2);
    v_y = lv_x * sin(x0(3)) + lv_y * sin(x0(3) + pi/2);

    omega = r/(3*L) * (u(1)+u(2)+u(3));
 
    %verify the radius
%     rad = sqrt(v_x*v_x + v_y*v_y) / omega
%     d = rad * 2
    
    x_record(:,i) = x0;
    v_record(:,i) = [v_x;
                     v_y;
                     omega];
    y_record(:,i) = y;
    
    %disturbance
    d = [normrnd(0,0.01); normrnd(0,0.01); normrnd(0,omega_variance)];
%     x1 = x0 + [ v_x * dt; v_y * dt; omega * dt]+ d;
    % without disturbance
    x0 = x0 + [ v_x * dt; v_y * dt; omega * dt];
end

% a = sqrt(v_record(1,:).^2 + v_record(2,:).^2)
samples_per_draw = 1;
trimmed_x_rec = x_record(:,1:samples_per_draw:end);
trimmed_v_rec = v_record(:,1:samples_per_draw:end);
trimmed_y_rec = y_record(:,1:samples_per_draw:end);


quiver(trimmed_x_rec(1,:),trimmed_x_rec(2,:),trimmed_v_rec(1,:),trimmed_v_rec(2,:));
quiver(trimmed_x_rec(1,:),trimmed_x_rec(2,:),cos(trimmed_x_rec(3,:)),sin(trimmed_x_rec(3,:)),'r*');

%quiver(trimmed_y_rec(1,:),trimmed_y_rec(2,:),zeros(n),zeros(n),'g*');
