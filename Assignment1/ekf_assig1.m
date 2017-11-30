%% Kalman Filter
clear all; clc;
close all;

makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

f = 10; % [Hz]
dt = 1 / f;% [s] timestep (update rate)

% robot sizing 
L = 0.3; % [m]
r = 0.25; % [m]

% initial state [x,y,theta]
x0 = [0 0 0]'; % [[m/s] [m/s] [rad/s]] intial state

% input
u = [-1.5 2 1]'; % [[rad/s] [rad/s] [rad/s]] inputs

% predicted mean and covariance
mu = [5 -5 1]'; % mean (mu)
% mu = [20 -20 1]'; % mean (mu)

S = eye(3);% covariance (Sigma)

% disturbance model
omega_std = 0.1 * pi / 180;
R = [0.01 0 0; 0 0.01 0; 0 0 (omega_std)].^2;

% Measurement model defined below
Q = [0.5 0 0; 0 0.5 0; 0 0 (10*pi/180)].^2;

% Simulation Initializations
Tf = 15; % duration
T = 0:dt:Tf; % time vector 
n = 3;
x = zeros(n,length(T));
x(:,1) = x0;
y = zeros(n,length(T));

% Variables to store during iteration
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
K_S = zeros(n,n,length(T));

mu_S (:,1) = mu; 

%% Main Loop
for t=2:length(T)
    %%% Simulation
    
    % Select a motion disturbance
%     e = RE*sqrt(Re)*randn(n,1);
    e = normrnd(0,[R(1,1);R(2,2);R(3,3)].^(1/2));
    % Update state
    x(:,t) = motion_model(x(:,t-1), u) + e;
    
    % Take measurement
    % Select a motion disturbance
    if(mod(t,10) == 0)
        %fprintf('perfect\n')
        Q = [0.01 0 0; 0 0.01 0; 0 0 (10*pi/180)].^(2);
    else
        %fprintf('normal\n')
        Q = [0.5 0 0; 0 0.5 0; 0 0 (10*pi/180)].^(2);
    end
    d = normrnd(0,[Q(1,1);Q(2,2);Q(3,3)].^(1/2));
    y(:,t) = measurement_model(x(:,t)) + d;
    
    %%% Extended Kalman Filter Estimation
    [mu, S, mup, K] = ekf(mu, S, y(:,t), ...
                          @motion_model, ...
                          @measurement_model, ...   
                          @linearized_motion_model, ...
                          @linearized_measurement_model, ...
                          Q, R, u);
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,:,t) = K;

    %% Plot results
    figure(1);clf; hold on;
    plot(y(1,1:t), y(2,1:t), 'g*');
    plot(x(1,1:t),x(2,1:t), 'ro--')
%     plot(5,-5,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    axis equal 
    if (makemovie) writeVideo(vidObj, getframe(gcf)); end

end
xlabel('xpos [m]');
ylabel('ypos [m]');
legend('Measurement','True State','State Estimate');
if (makemovie) close(vidObj); end

figure
subplot (3, 1, 1); 
hold on;
plot(T, y(1,:), 'g*--');
plot(T, x(1, :), 'ro--');
plot(T, mu_S(1,:), 'bx--');
ylabel('x_1 - X Position (m)');
xlabel('Time (s)');
legend('Measurement','True State','State Estimate');

subplot(3, 1, 2) 
hold on;
plot(T, y(2,:), 'g*--');
plot(T, x(2, :), 'ro--');
plot(T, mu_S(2,:), 'bx--');
ylabel('x_2 - Y Position (m)');
xlabel('Time (s)');

subplot(3, 1, 3) 
hold on;
plot(T, y(3,:), 'g*--');
plot(T, x(3, :), 'ro--');
plot(T, mu_S(3,:), 'bx--');
ylabel('x_3 - Theta (deg)');
xlabel('Time (s)');