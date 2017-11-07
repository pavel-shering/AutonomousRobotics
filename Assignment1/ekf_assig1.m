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
dt = 0.1;% [s] timestep (update rate)

% robot sizing 
L = 0.3; % [m]
r = 0.25; % [m]
f = 10; % [Hz]

% initial state [x,y,theta]
x0 = [0 0 0]'; % [[m/s] [m/s] [rad/s]] intial state

% input
u = [-1.5 2 1]'; % [[rad/s] [rad/s] [rad/s]] inputs

% predicted mean and covariance
mu = [1 2 0.5]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% disturbance model
omega_std = 0.1 / 180 * pi;
R = [0.01 0 0; 0 0.01 0; 0 0 (omega_std)].^2;
[RE, Re] = eig (R);

% Measurement model defined below
Q = [0.5 0 0; 0 0.5 0; 0 0 10 / 180 * pi] .^2;
[QE, Qe] = eig (Q);

% Simulation Initializations
Tf = 10; % duration
T = 0:dt:Tf; % time vector 
n = 3;
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));

% figure(1); clf; hold on;
% 
% x_record = zeros(3, n);
% v_record = zeros(3, n);
% y_record = zeros(3, n);

% Variables to store during iteration
x_history = zeros(n, length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
K_S = zeros(n,n,length(T));

%% linearizing g(x_t-1, u_t)
syms x1 x2 x3 u1 u2 u3 

% local frame velocities
lv_x = (r/3) * (u1*cos(pi) + u2*cos(-pi/3) + u3*cos(pi/3));
lv_y = (r/2) *  (u2*sin(-pi/3) + u3*sin(pi/3));

% globbal fram velocities 
v_x = lv_x * cos(x3) + lv_y * cos(x3 + pi/2);
v_y = lv_x * sin(x3) + lv_y * sin(x3 + pi/2);

omega = r/(3*L) * (u1 + u2 + u3);
x_arr = [x1;x2;x3];

g = x_arr + [ v_x * dt; v_y * dt; omega * dt];

G = jacobian(g, [x1; x2; x3]);

%% Main Loop
for t=2:length(T)
    %%% Simulation
    
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = motion_model(x(:,t-1),u) + e;
    
    % Take measurement
    
    % Select a motion disturbance
    d = QE*sqrt(Qe)*randn(n,1);
    % Determine measurement  
    y(:,t) = x(:,t-1) + d;
    
    %%% Extended Kalman Filter Estimation
    [mu, S, mup, K] = ekf(mu, S, y(:,t), ...
                          @motion_model, ...
                          @measurement_model, ...   
                          @linearized_motion_model, ...
                          @linearized_measurement_model, ... @airplane_radar_linearized_measurement_model, ...
                          Q, R, u);
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
%     K_S(:,t) = K;

    %%% Plot results
    figure(1);clf; hold on;
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1],[0 0],'b--')
    plot(x(1,2:t),x(3,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(3,2:t), 'bx--')
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    title('True state and belief')

    axis equal
    axis([-1 20 -10 10])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
xlabel('x_pos');
ylabel('y_pos');
legend('True State', 'Belief');
if (makemovie) close(vidObj); end

figure
plot(x(1, :), x(2, :));
hold on
plot(mu_S(1, :), mu_S(2, :));
% axis([-1 4 -2 2]);
