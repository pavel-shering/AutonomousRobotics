clear all;
close all;
clc;
%% Planning map


I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Pad the map
pad_radius = 4;
padded_map = pad_map(map, pad_radius);

% Robot start position
dxy = 0.1;
startpos = [40 5 pi];

% Target location
searchgoal = [50 10];

% Plotting
figure(1); clf; hold on;
colormap('gray');
drawn_map = map + padded_map * 0.3;
imagesc(1-drawn_map');
plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1)/dxy, searchgoal(2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal

%% Problem parameters
tic;

% Set up the map
xMax = [926 716]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = startpos(1:2)*10;
xF = searchgoal *10;

%% Multi-query PRM, created in batch
tic;

% Get milestones
nS = 450;
nBS = 100;
nRS = nS - nBS;
sigma = 50;

%samples = ceil([xR(1)*rand(nS,1)+xMin(1) xR(2)*rand(nS,1)+xMin(2)]);
samples = zeros(nS,2);
num_B_samples = 0;
num_R_samples = 0;
num_samples = 0;
while num_samples < nS
    q = ceil([xR(1)*rand(1,1)+xMin(1) xR(2)*rand(1,1)+xMin(2)]);
    q2 = round (q + [normrnd(0,sigma) normrnd(0,sigma)]);
    if ~(q2(1) > xMin(1) && q2(1) < xMax(1) && q2(2) > xMin(2) && q2(2) < xMax(2))
        continue;
    end
    if num_B_samples < nBS && padded_map(q(1),q(2)) &&  padded_map(q2(1),q2(2))
        q_avg = round((q + q2)/2);
        if ~padded_map(q_avg(1),q_avg(2))
            samples(num_samples + 1, :) = q_avg;
            num_B_samples = num_B_samples + 1;
        end
    elseif num_R_samples < nRS
        if ~padded_map(q(1),q(2)) &&  ~padded_map(q2(1),q2(2))
            samples(num_samples + 1, :) = q;
            num_R_samples = num_R_samples + 1;
        end
    end
	num_samples = num_B_samples + num_R_samples;
%%working lavalle sampling
%     if padded_map(q(1),q(2)) &&  ~padded_map(q2(1),q2(2))
%         num_samples = num_samples + 1;
%         samples(num_samples, :) = q2;
%     elseif ~padded_map(q(1),q(2)) &&  padded_map(q2(1),q2(2))
%         num_samples = num_samples + 1;
%         samples(num_samples, :) = q;       
%     end
end

keep = zeros(nS);
for i = 1:nS
    keep(i) = ~padded_map(samples(i,1),samples(i,2));
end
milestones = [x0; xF; samples(find(keep==1),:)];
figure(1); hold on;
plot(samples(:,1),samples(:,2),'k.');
plot(milestones(:,1),milestones(:,2),'m.');
nM = length(milestones(:,1));
disp('Time to generate milestones');
toc;

% Attempt to add closest p edges
tic;
p = 20;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
for i = 1:nM
    % Find closest neighbours
    for j = 1:nM
        d(j) = norm(milestones(i,:)-milestones(j,:));
    end
    [d2,ind] = sort(d);
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:p
        cur = ind(j);
        if (i<cur)
            collision = false;
            [line_occupancy_x, line_occupancy_y] = bresenham(milestones(i,1), milestones(i,2), milestones(cur,1), milestones(cur,2));
            for k = 1:length(line_occupancy_x)
                if (padded_map(line_occupancy_x(k),line_occupancy_y(k)))
                    collision = true;
                    break;
                end
            end
            if (~collision)
                e(i,cur) = 1;
                e(cur,i) = 1;
                plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
            end
        end
    end
end
disp('Time to connect roadmap');
toc;

% Find shortest path
tic;
[sp, sd] = shortestpath_mr(milestones, e, 1, 2, 1, 1, 0);
disp('Time to find shortest path');
toc;

waypoints = zeros (length(sp),2);
for i=1:length(sp)-1
    plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
    waypoints(i,:) = milestones(sp(i),:);
end
waypoints(length(sp),:) = milestones(sp(length(sp)),:);
waypoints = waypoints * 0.1;
