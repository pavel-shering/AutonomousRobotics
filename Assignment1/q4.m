clear all
close all

%% Question 2 : Motion Model (Matrix form)
    r = 0.25; % wheel radius [m]
    l = 0.3; % robot frame radius [m]
    sample_time = 0.1; % time between samples [s]
    
    syms x1 x2 x3 u1 u2 u3;
    
    j = [-1/3  cosd(60)/3 cosd(60)/3;
            0 sind(-60)/2 sind(60)/2;
      1/(3*l)     1/(3*l)     1/(3*l)] .* (sample_time * r);
  
    k = [cos(x3) cos(x3 + pi/2) 0;
         sin(x3) sin(x3 + pi/2) 0;
            0        0         1];
        
    g = [x1; x2; x3] + k*j*[u1; u2; u3];
    
%     g = [x1 + (-u1/3 + u2 * cosd(60)/3 + u3 * cosd(60)/3) * cos(x3) - (u2 * sind(-60)/2 + u3 * sind(60)/2) * sin(x3);
%          x2 + (-u1/3 + u2 * cosd(60)/3 + u3 * cosd(60)/3) * sin(x3) + (u2 * sind(-60)/2 + u3 * sind(60)/2) * cos(x3);
%          x3 + u1 / (3*l) + u2 / (3*l) + u3 / (3*l)];
     
    G = jacobian(g,  [x1; x2; x3])
    
%% Question 3 : Measurement Model
% x = [0 0 0];
% measurement_model = [x(1); x(2); x(3)-99.7/180*pi];
% measurement_model_disturbance = [randn(1)*0.5; randn(1)*0.5; randn(1)*10/180*pi];
H = [1 0 0;
    0 1 0;
    0 0 1];

%% Question 4: Extended Kalman Filter
dt = 0.1;
Tf = 15;
T = 0:dt:Tf;

R = [0.01 0 0; 0 0.01 0; 0 0 0.1*pi/180] .^2;
Q = [0.5 0 0; 0 0.5 0; 0 0 10/180*pi] .^2;

x = [1; 1; pi/2];
u = [-1.5; 2; 1];

mup = [1; 1; pi/2];
mu = [1; 2; 0.5];

S = 1*eye(3);

x_history = zeros(3, length(T));
mu_history = zeros(3, length(T));

for t = 1 : length(T)
    u(2) = u(2) - 0.01;
    u(3) = -u(2);
    % Calculate error and disturbance
    e = [randn(1) 0 0; 0 randn(1) 0; 0 0 randn(1)] * (sqrt(R) * [1; 1; 1]);
    d = [randn(1) 0 0; 0 randn(1) 0; 0 0 randn(1)] * (sqrt(Q) * [1; 1; 1]);
    
    % Calculate motion model and measurement model measurement
    x = motion_model(x, u) + e;
    y = [x(1); x(2); x(3)-99.7/180*pi] + d;
    
    
    %%% Kalman Filter Prediction
    
    % Propagate mu through the nonlinear motion model
    mup = motion_model(mu,u);
    
    % Linearized motion model at the predicted mean
    x1 = mup(1);
    x2 = mup(2);
    x3 = mup(3);
    u1 = u(1);
    u2 = u(2);
    u3 = u(3);
    G_t = double(vpa(subs(G)));
    
    % Compute predicted covariance
    Sp = G_t*S*G_t' + R;
    
    %%% Measurement update
    % Linearized measurement model at the predicted mean
    H_t = [mup(1) 0 0;
           0 mup(2) 0;
           0 0 mup(3)];
    
    % Compute Kalman gain
    Kg = Sp*H_t'*inv(H_t*Sp*H_t'+Q);
    
    % Update mean using the nonlinear measurement model
    mu = mup + Kg*(y-[mup(1); mup(2); mup(3)-99.7/180*pi]);
    
    % Update the covariance based on the measurement model
    S = (eye(length(mu))-Kg*H_t)*Sp;
    
    mu_history(:,t) = mu;
    x_history(:,t) = x;
end

figure
plot(x_history(1, :), x_history(2, :));
hold on
plot(mu_history(1, :), mu_history(2, :));
axis([-1 4 -2 2]);

