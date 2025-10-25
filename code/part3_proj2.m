%% inital settings
close all; clear; clc;

%% get CB for A and B1

% Suppose Maximum radius of boundary point moving is R=sqrt(65)

delta_grid = 1;
% Define total grid, where physical grid is 32x32:
GridR = 32/delta_grid; % Dimension of cells in grid, 32, 64 = 32/desired delta

% Robot A inital placement and target
dA = [4.5;24.5];
D = 1;
R = D / 2;     % Radius
dT = [4.5;8.5];

% Obstacle B1 vertices points
b1_B1 = [0;18];
b2_B1 = [10;18];
b3_B1 = [10;19];
b4_B1 = [0;19];

B1_pts = [b1_B1, b2_B1, b3_B1, b4_B1];

polyshape_vert_w_circles = get_CB_disk(D, B1_pts);
% 
% figure(); plot(polyshape_vert_w_circles);
% axis equal;

%% Do it for all CB

% Obstacle B2 vertices points
b1_B2 = [17;17];
b2_B2 = [18;17];
b3_B2 = [18;29];
b4_B2 = [17;29];

B2_pts = [b1_B2, b2_B2, b3_B2, b4_B2];

% Obstacle B3 vertices points
b1_B3 = [25;18];
b2_B3 = [32;18];
b3_B3 = [32;19];
b4_B3 = [25;19];

B3_pts = [b1_B3, b2_B3, b3_B3, b4_B3];

% Obstacle B4 vertices points
b1_B4 = [0;14];
b2_B4 = [19;14];
b3_B4 = [19;15];
b4_B4 = [0;15];

B4_pts = [b1_B4, b2_B4, b3_B4, b4_B4];

% Obstacle B5 vertices points
b1_B5 = [24;13];
b2_B5 = [32;13];
b3_B5 = [32;15];
b4_B5 = [24;15];

B5_pts = [b1_B5, b2_B5, b3_B5, b4_B5];

% Obstacle B6 vertices points
b1_B6 = [10;19];
b2_B6 = [12;19];
b3_B6 = [12;20];
b4_B6 = [10;20];

B6_pts = [b1_B6, b2_B6, b3_B6, b4_B6];

% Obstacle B7 vertices points
b1_B7 = [23;19];
b2_B7 = [25;19];
b3_B7 = [25;20];
b4_B7 = [23;20];

B7_pts = [b1_B7, b2_B7, b3_B7, b4_B7];

% Obstacle BO1 vertices points
b1_BO1 = [0;29];
b2_BO1 = [32;29];
b3_BO1 = [32;30];
b4_BO1 = [0;30];

B_BO1_pts = [b1_BO1, b2_BO1, b3_BO1, b4_BO1];

% Obstacle BO2 vertices points
b1_BO2 = [0;0];
b2_BO2 = [1;0];
b3_BO2 = [1;30];
b4_BO2 = [0;30];

B_BO2_pts = [b1_BO2, b2_BO2, b3_BO2, b4_BO2];

% Obstacle BO3 vertices points
b1_BO3 = [0;0];
b2_BO3 = [32;0];
b3_BO3 = [32;1];
b4_BO3 = [0;1];

B_BO3_pts = [b1_BO3, b2_BO3, b3_BO3, b4_BO3];

% Obstacle BO4 vertices points
b1_BO4 = [31;0];
b2_BO4 = [32;0];
b3_BO4 = [32;30];
b4_BO4 = [31;30];

B_BO4_pts = [b1_BO4, b2_BO4, b3_BO4, b4_BO4];

Obstacles = zeros(2,4,11);
Obstacles(:,:,1) = B1_pts;
Obstacles(:,:,2) = B2_pts;
Obstacles(:,:,3) = B3_pts;
Obstacles(:,:,4) = B4_pts;
Obstacles(:,:,5) = B5_pts;
Obstacles(:,:,6) = B6_pts;
Obstacles(:,:,7) = B7_pts;
Obstacles(:,:,8) = B_BO1_pts;
Obstacles(:,:,9) = B_BO2_pts;
Obstacles(:,:,10) = B_BO3_pts;
Obstacles(:,:,11) = B_BO4_pts;

nObst = size(Obstacles,3);

% Save CB_theta0 for each obstacle, for each layer
CB_theta0_Poly_list = cell(nObst, 1);

for i = 1:nObst
    polyshape_vert_w_circles = get_CB_disk(D, Obstacles(:,:,i));
    CB_theta0_Poly_list{i} = polyshape_vert_w_circles;
    
    % Merge CB's
    if i > 1
        CB_rn = union(CB_theta0_Poly_list{i},CB_theta0_Poly_list{i-1});
        if i == 2
            CB_tot = CB_rn;
        else
            CB_tot = union(CB_rn,CB_tot);
        end
    end

end

% Plot it nicely:
figure;

% Print Obstacle & Mark are of CB of each obstacle:
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);

    % Plot Obstacle B
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
        'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
    hold on;
end

% Plot the polyshape
plot(CB_tot, 'FaceColor', [1 0.6 0.6], 'EdgeColor', 'r', 'LineWidth', 1.5);

% Draw circle:

% Create circle points
theta = linspace(0, 2*pi, 100);
x_disk = dA(1) + R * cos(theta);
y_disk = dA(2) + R * sin(theta);

% Plot the disk
fill(x_disk, y_disk, 'b', 'FaceAlpha', 0.5);  % Blue disk with transparency
% Add "S" label
text(dA(1)-1, dA(2)-1, 'S', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');
% Add "T" label
text(dT(1)-1, dT(2)-1, 'T', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');


% Apply zoom if specified
xlim([-1,33]);
ylim([-1,31]);

% Final touches
grid on;
hold on; axis equal;
set(gcf, 'Renderer', 'painters');
xlabel('X'); ylabel('Y');

title('CB for Robot Disk');
exportgraphics(gcf, 'RobotDISK_CB_tot.png',...
            'Resolution', 300);


%% Now, make the discritized map

% Define total grid, where physical grid is 32x32:
grid_map = zeros(GridR,GridR); % 1's for CB itself

% Generate discrete grid when CB is intersecting the pixels

[boundary_CB_x,boundary_CB_y] = boundary(CB_tot);

% Firstly, For each layer get the boundarys of the inner-holes

% Find rows with NaNs (they separate boundary sections)
nanIdx = find(isnan(boundary_CB_x(:,1)));

% Define Sections start-end
n_sec = length(nanIdx)+1; % number of sections
sectionStarts = zeros(n_sec,1);
sectionEnds = zeros(n_sec,1);

for q = 1:n_sec
    if q == 1
        sectionStarts(q) = 1;
    else
        sectionStarts(q) = nanIdx(q-1)+1;
    end
    
    if q ~= n_sec
        sectionEnds(q) = nanIdx(q)-1;
    else
        sectionEnds(q) = size(boundary_CB_x,1);
    end
end

points_x = boundary_CB_x(sectionStarts(2):sectionEnds(2), 1);
points_y = boundary_CB_y(sectionStarts(2):sectionEnds(2), 1);

for xi = 0:GridR-1
    for yi = 0:GridR-1
        
        [inFree,~] = inpolygon(xi+0.5,yi+0.5,points_x,points_y);
        

        % If the cell didnt get marked AND it overlaps (with area) CB
        if not(inFree)
            grid_map(yi+1,xi+1) = 1;
        end
    end
end 

figure;

domain_size = GridR * delta_grid;

hold on;
axis equal;
axis([0 domain_size 0 domain_size]);
set(gca,'xtick',0:delta_grid:domain_size,'ytick',0:delta_grid:domain_size);

grid on;
box on;

% Draw filled blocks for grid_Layer_i == 1 or 2
for i = 1:GridR
    for j = 1:GridR
        x = (j-1) * delta_grid;
        y = (i-1) * delta_grid;

        if grid_map(i,j) == 1
            % Draw black square
            fill([x x+delta_grid x+delta_grid x], ...
                 [y y y+delta_grid y+delta_grid], ...
                 'k');
        end
    end
end

% Draw grid lines manually
for i = 0:delta_grid:domain_size
    % Horizontal lines
    plot([0 domain_size], [i*delta_grid i*delta_grid], 'k');
    % Vertical lines
    plot([i*delta_grid i*delta_grid], [0 domain_size], 'k');
end

xlabel('X');
ylabel('Y');
title('CB for Robot Disk Discretized');
% Export image
exportgraphics(gcf, 'RobotDISK_CB_tot_Discretized.png', 'Resolution', 300);

%% Store free-space nodes

% Saves collums of [row,col,lyr] of free-spaces
free_space_nodes = [];
% Saves collums of [x,y,theta] of free-spaces
free_space_nodes_xyth = [];

% Also, get S and T in node corrdinates:
S_xyth = [dA;0];
T_xyth = [dT;0];
S_Cor = [24+1;4+1;0];
T_cor = [8+1;4+1;0];
S = -1;
T = -1;


% First, store all free-space without connections 

% Matrix - mark 0 as Free-space, rest is 1

for row = 1:GridR
    for col = 1:GridR
        if grid_map(row,col) == 0 % Free-space

            free_space_nodes = [free_space_nodes,[row;col;0]];

            if isequal(S_Cor,[row;col;0])
                S = size(free_space_nodes,2);
            end
            if isequal(T_cor,[row;col;0])
                T = size(free_space_nodes,2);
            end

            % Free-space is middle of pixel in XY grid
            free_space_nodes_xyth = ...
                [free_space_nodes_xyth,...
                [(col-1)*delta_grid+0.5*delta_grid;...
                (row-1)*delta_grid+0.5*delta_grid;0]];

        end
    end
end


n_freespace = size(free_space_nodes,2);

%% Now, make graph matrix

% Graph has row as node and collom as [neighbor index;l_b] of neighbors
% and theirl_b

graph = cell(n_freespace,1);

for i=1:n_freespace
    
    % Display progress
    percent_done = (i / n_freespace) * 100;
    fprintf('\rProgress: %6.2f%%', percent_done);

    row_cur = free_space_nodes(1,i);
    col_cur = free_space_nodes(2,i);
    lay_cur = free_space_nodes(3,i);

    % In general - 4 neighbors in 2D grid, will delete some l8tr
    neighbors_idx = [[row_cur-1;col_cur;lay_cur],[row_cur;col_cur-1;lay_cur],...
        [row_cur+1;col_cur;lay_cur],[row_cur;col_cur+1;lay_cur]];
    idx_delet = [0,0,0,0]; % 0-keep, 1-delete
    

    % Fix indecies to be inside matrix 
    % & check neighbor to be in free-space
    for q = 1:4
        rowMat_q = neighbors_idx(1,q);
        colMat_q = neighbors_idx(2,q);
        lay_cur_q = neighbors_idx(3,q);
        
        % Remove neighbors that have non-relevant indices
        if rowMat_q<1 || rowMat_q>GridR ...
               || colMat_q<1 || colMat_q>GridR
            idx_delet(q) = 1;
        % Check if the neighbor is even in free-space:
        elseif grid_map(rowMat_q,colMat_q) == 1
            idx_delet(q) = 1;
        end
        
    end

    % Keep specific neighbors:
    neighbors_idx_keep = [];
    for k = 1:4
        if idx_delet(k) == 0
            neighbors_idx_keep = [neighbors_idx_keep,neighbors_idx(:,k)];
        end
    end

    neighbors_idx = neighbors_idx_keep; % Assign the remaining neighbors

    % Now that we have neighbors of this free-cell, calculate l_b
    
    if isempty(neighbors_idx) % If there are no neighbors, finish
        continue;
    end

    % Go over each neighbor one-by-one:
    for q = 1:size(neighbors_idx,2)

        for j = 1:n_freespace % get j as index of neighbor in free_space_nodes
            if isequal(free_space_nodes(:,j),neighbors_idx(:,q))
                break;
            end
        end
        
        % Now, calculate "distance" between node_j and node_i
        
        node_j = free_space_nodes_xyth(:,j);
        node_i = free_space_nodes_xyth(:,i);

        l_b= normVectorZTheta(node_i,node_j);
        
        if isempty(graph{i})
            graph{i} = [j;l_b];
        else
            graph{i}(:, end+1) = [j;l_b];
        end

    end

end

%% Run A* algorithm on freesp_lay_connections
% In here, we will save points as (row,col,layer_theta)

vertss = free_space_nodes_xyth;
[path,C,prices] = Astar(graph, vertss, S, T);

l_path = 0;
if not(isempty(prices))
    l_path = sum(prices);
end

%% Print 3D path

% Plot
figure;

X_path = free_space_nodes_xyth(1,path);
Y_path = free_space_nodes_xyth(2,path);
Z_path = free_space_nodes_xyth(3,path);
plot3(X_path,Y_path,Z_path);

% Print Obstacle & Mark are of CB of each obstacle:
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot Obstacle B
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
	    'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end
grid on;
xlabel('X');
ylabel('Y');
% set(gca, 'YDir','reverse')
zlabel('Z');
view(2);
title('3D Points from Path Array');
axis equal;

exportgraphics(gcf, 'RobotDISK_Path_xy.png', 'Resolution', 300);

%% Print 3D closed nodes

% Plot
figure;

X_path = free_space_nodes_xyth(1,C(1,:));
Y_path = free_space_nodes_xyth(2,C(1,:));
Z_path = free_space_nodes_xyth(3,C(1,:));
plot3(X_path,Y_path,Z_path,'.');

% Print Obstacle & Mark are of CB of each obstacle:
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot Obstacle B
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
	    'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end
grid on;
xlabel('X');
ylabel('Y');
% set(gca, 'YDir','reverse')
zlabel('Z');
view(2);
title('3D Points from Closed Nodes Array');
axis equal;

hold on;
plot3(S_xyth(1), S_xyth(2), S_xyth(3), 'ro', 'MarkerSize', 8, 'DisplayName','S')
plot3(T_xyth(1), T_xyth(2), T_xyth(3), 'bo', 'MarkerSize', 8, 'DisplayName','T')


exportgraphics(gcf, 'RobotDISK_C_xy.png', 'Resolution', 300);

%% Print robot doing the path:

% Parameters
Time = 10;  % total video time in seconds
num_frames = length(path); 

% number of steps in the robot's path (including begging and end)
dt = Time / num_frames;  % time step to match video duration

% Setup figure and video writer
figure();
set(gcf, 'Renderer', 'opengl'); % 'opengl' usually better for getframe
axis equal;
hold on;
grid on;
xlim([-1,33]);
ylim([-1,31]);
xlabel('X'); ylabel('Y');
title('Robot Silhouette ');

% Plot Obstacles once
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
        'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end

% Silhouette color
silhouette_color = [0.2 0.6 1];  % soft blue

% === Change to MP4 writer ===
v = VideoWriter('robotDISK_path_silhouette.mp4', 'MPEG-4');  % <-- MP4 output
v.FrameRate = 1 / dt;
open(v);

% Preallocate patch handles for silhouette polygons to improve speed
silhouette_patches = gobjects(num_frames,1);

for i = 1:num_frames
    % Get current robot position and orientation
    x = free_space_nodes_xyth(1, path(i));
    y = free_space_nodes_xyth(2, path(i));
    
    theta = linspace(0, 2*pi, 100);
    x_disk = R * cos(theta);
    y_disk = R * sin(theta);

    A_trans = [x_disk;y_disk] + [x; y];

    % Plot silhouette polygons cumulatively (keep them)
    silhouette_patches(i) = patch(A_trans(1,:), A_trans(2,:), silhouette_color, ...
                              'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
    
    % Plot current robot (highlighted)
    h = patch(A_trans(1,:), A_trans(2,:), 'r', 'EdgeColor', 'k');

    drawnow; % Ensure figure updates for getframe

    % Capture frame
    frame = getframe(gcf);
    writeVideo(v, frame);

    % Delete current robot highlight (keep silhouette)
    delete(h);
end

% Add "S" label
text(dA(1)-1, dA(2)-1, 'S', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');
% Add "T" label
text(dT(1)-1, dT(2)-1, 'T', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');

close(v);
exportgraphics(gcf, 'RobotDISK_Silhouette.png', 'Resolution', 300);

%% Run A* algorithm ONLINE OPTION 1
% In here, we will save points as (row,col,layer_theta)

vertss = free_space_nodes_xyth;
[path,C,List_FromCurrToNewBest] = AstarOnline1(graph, vertss, S, T);

% Now, create the real path:
RealPath = [];

for i = 1:length(List_FromCurrToNewBest)
    segment_i = List_FromCurrToNewBest{i};
    if i ~= 1
        segment_i = segment_i(2:end);
    end
    for j = 1:length(segment_i)
        RealPath = [RealPath,segment_i(j)];
    end
end

%% Print 3D path

% Plot
figure;

X_path = free_space_nodes_xyth(1,RealPath);
Y_path = free_space_nodes_xyth(2,RealPath);
Z_path = free_space_nodes_xyth(3,RealPath);
plot3(X_path,Y_path,Z_path);

% Print Obstacle & Mark are of CB of each obstacle:
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot Obstacle B
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
	    'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end
grid on;
xlabel('X');
ylabel('Y');
% set(gca, 'YDir','reverse')
zlabel('Z');
view(2);
title('3D Points from Path Array');
axis equal;

% Add "S" label and marker
plot(dA(1), dA(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % Green circle
text(dA(1)-1, dA(2)-1, 'S', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');

% Add "T" label and marker
plot(dT(1), dT(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);  % Magenta circle
text(dT(1)-1, dT(2)-1, 'T', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');

exportgraphics(gcf, 'RobotDISK_Path_Online1.png', 'Resolution', 300);

%% Print robot doing the path:

% Setup figure and video writer
figure();
set(gcf, 'Renderer', 'opengl'); % 'opengl' usually better for getframe
axis equal;
hold on;
grid on;
xlim([-1,33]);
ylim([-1,31]);
xlabel('X'); ylabel('Y');
title('Robot Silhouette ');

% Plot Obstacles once
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
        'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end

% Silhouette color
silhouette_color = [0.2 0.6 1];  % soft blue
ST_color = [1 0 0]; % red

for i = 1:length(RealPath)
    % Get current robot position and orientation
    x = free_space_nodes_xyth(1, RealPath(i));
    y = free_space_nodes_xyth(2, RealPath(i));
    theta = linspace(0, 2*pi, 100);
    x_disk = R * cos(theta);
    y_disk = R * sin(theta);

    A_trans = [x_disk;y_disk] + [x; y];
    if ~isequal([x;y],dA)
        if i == length(RealPath)
            % Plot silhouette polygons cumulatively (keep them)
            patch(A_trans(1,:), A_trans(2,:), ST_color, ...
                                  'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
        else
            % Plot silhouette polygons cumulatively (keep them)
            patch(A_trans(1,:), A_trans(2,:), silhouette_color, ...
                                  'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
        end
    else
        % Plot silhouette polygons cumulatively (keep them)
        patch(A_trans(1,:), A_trans(2,:), ST_color, ...
            'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
    end   
    
end

% Add "S" label
text(dA(1)-1, dA(2)-1, 'S', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');
% Add "T" label
text(dT(1)-1, dT(2)-1, 'T', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');

close(v);
exportgraphics(gcf, 'RobotDISKOnline1_Silhouette.png', 'Resolution', 300);

%% Run A* algorithm ONLINE OPTION 2
% In here, we will save points as (row,col,layer_theta)

vertss = free_space_nodes_xyth;
[path,C,List_FromCurrToNewBest] = AstarOnline2(graph, vertss, S, T);

% Now, create the real path:
RealPath = [];

for i = 1:length(List_FromCurrToNewBest)
    segment_i = List_FromCurrToNewBest{i};
    if i ~= 1
        segment_i = segment_i(2:end);
    end
    for j = 1:length(segment_i)
        RealPath = [RealPath,segment_i(j)];
    end
end

%% Print 3D path

% Plot
figure;

X_path = free_space_nodes_xyth(1,RealPath);
Y_path = free_space_nodes_xyth(2,RealPath);
Z_path = free_space_nodes_xyth(3,RealPath);
plot3(X_path,Y_path,Z_path);

% Print Obstacle & Mark are of CB of each obstacle:
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot Obstacle B
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
	    'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end
grid on;
xlabel('X');
ylabel('Y');
% set(gca, 'YDir','reverse')
zlabel('Z');
view(2);
title('3D Points from Path Array');
axis equal;

% Add "S" label and marker
plot(dA(1), dA(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % Green circle
text(dA(1)-1, dA(2)-1, 'S', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');

% Add "T" label and marker
plot(dT(1), dT(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);  % Magenta circle
text(dT(1)-1, dT(2)-1, 'T', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');

exportgraphics(gcf, 'RobotDISK_Path_Online2.png', 'Resolution', 300);

%% Print robot doing the path:

% Setup figure and video writer
figure();
set(gcf, 'Renderer', 'opengl'); % 'opengl' usually better for getframe
axis equal;
hold on;
grid on;
xlim([-1,33]);
ylim([-1,31]);
xlabel('X'); ylabel('Y');
title('Robot Silhouette ');

% Plot Obstacles once
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
        'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
end

% Silhouette color
silhouette_color = [0.2 0.6 1];  % soft blue
ST_color = [1 0 0]; % red

for i = 1:length(RealPath)
    % Get current robot position and orientation
    x = free_space_nodes_xyth(1, RealPath(i));
    y = free_space_nodes_xyth(2, RealPath(i));
    theta = linspace(0, 2*pi, 100);
    x_disk = R * cos(theta);
    y_disk = R * sin(theta);

    A_trans = [x_disk;y_disk] + [x; y];
    if ~isequal([x;y],dA)
        if i == length(RealPath)
            % Plot silhouette polygons cumulatively (keep them)
            patch(A_trans(1,:), A_trans(2,:), ST_color, ...
                                  'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
        else
            % Plot silhouette polygons cumulatively (keep them)
            patch(A_trans(1,:), A_trans(2,:), silhouette_color, ...
                                  'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
        end
    else
        % Plot silhouette polygons cumulatively (keep them)
        patch(A_trans(1,:), A_trans(2,:), ST_color, ...
            'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1);
    end   
    
end

% Add "S" label
text(dA(1)-1, dA(2)-1, 'S', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');
% Add "T" label
text(dT(1)-1, dT(2)-1, 'T', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');

close(v);
exportgraphics(gcf, 'RobotDISKOnline2_Silhouette.png', 'Resolution', 300);