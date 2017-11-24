clear all;
close all;
clc;

makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

f = 10; % [Hz]
dt = 1 / f;% [s] timestep (update rate)

%simulation
T = 20; % [s] Duration
dt = 0.1;% [s] timestep (update rate)
tvec = 0:dt:T; % Time vector
n = length(tvec); % Number of timesteps

% robot dimensions 
L = 0.3; % [m]
d_lim = 30*pi/180;% [rad] angle limit

% initial state [x,y,theta]
x0 = [0 0]'; % [[m/s] [m/s] [rad/s]] intial state

%inputs 
% u = [3 (10-t)*pi/180]';% [ [m/s] [rad/s]]




% xlabel('xpos [m]');
% ylabel('ypos [m]');
% legend('Measurement','True State','State Estimate');
% if (makemovie) close(vidObj); end
