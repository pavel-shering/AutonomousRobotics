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

run('IGVCmap.m');

f = 10; % [Hz]
dt = 1 / f;% [s] timestep (update rate)

%simulation
T = 110; % [s] Duration
tvec = 0:dt:T; % Time vector
n = length(tvec); % Number of timesteps

% robot dimensions 
L = 0.3; % [m]
d_lim = 30*pi/180;% [rad] angle limit

% carrot controller parameters
r = 1.2;


% initial state [x,y,theta]
t = 0;
xprev = startpos'; %[0 0 0]'; % [[m/s] [m/s] [rad/s]] intial state
u = [3 0]';

figure('Name','Robot Navigation');
hold on;

x_record = zeros(3, n);
u_record = zeros(2, n);
c_record = zeros(2, n);

omega_variance = 1 / 180 * pi;

%waypoints = [10 -2.5; -10 -2.5; -10 2.5; 10 2.5];
waypoint_index = 2;
prev_waypoint = waypoints(1,:);
for i = 1:n
    t = t + dt;
    %input 
   % u = [3 (10-t)*pi/180]';% [ [m/s] [rad/s]]
    p1 = prev_waypoint;
    p2 = waypoints(waypoint_index,:);
    [outside carrot_outside carrot_point] = get_carrot(p1', p2', xprev(1:2), r);
    
   
    u = p_control(carrot_point, xprev, u);
    if (carrot_outside)
        
        prev_waypoint = waypoints(waypoint_index,:);
        waypoint_index = waypoint_index + 1;
    end
    x_record(:,i) = xprev;
    u_record(:,i) = u;
    c_record(:,i) = carrot_point;
    
    %disturbance
    d = [normrnd(0,0.02); normrnd(0,0.02); normrnd(0,omega_variance)];
    
    xcur =  bicycle(xprev,  u(1), u(2), L, dt) + d;
    xprev = xcur;
    
    
    if (waypoint_index > length(waypoints))
        
%         waypoint_index = 1
        break;
    end
end

colormap('gray');
drawn_map = map + padded_map * 0.3;
imagesc(1-drawn_map');
plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1)/dxy, searchgoal(2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );

x_record(1:2,:) = x_record(1:2,:)*10; %%%% Massive hack, fix this

quiver(x_record(1,:),x_record(2,:),cos(x_record(3,:)),sin(x_record(3,:)),'r*');
quiver(x_record(1,:),x_record(2,:),cos(x_record(3,:)+u_record(2,:)), ...
    sin(x_record(3,:)+u_record(2,:)),'g*');
% plot(c_record(1,:),c_record(2,:),'bo');

xlabel('xpos [dm]');
ylabel('ypos [dm]');
axis equal;


% xlabel('xpos [m]');
% ylabel('ypos [m]');
% legend('Measurement','True State','State Estimate');
% if (makemovie) close(vidObj); end
