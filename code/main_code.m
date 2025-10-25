%% inital settings
close all; clear; clc;

%% Method of discretization and free-space pixel check position

% Method 1 - for discretized map using boundary of CB
% Method 2 - for discretized map from checking free-space of point in pixel
Method = 2;

% Option 1 - choose the free-space point checker as the bottom-left
% of the pixel
% Option 2 - choose it as the middle of the pixel
Free_pixel_check = 2;

% Prevent (0) or keep (1) cyclic of nodes in theta dir
% Cyclic = 0;

%% Check grid:
% N = 20;         % Grid size
% a = 10;         % Starting point (a, a)
% b = 8;          % Length of line
% angles = 45:45:360-45;
% angles = 5:5:5;
% 
% for th = angles
%     figure();
%     clf;
%     theta_rad = deg2rad(th);
%     P1 = [a, a];
%     P2 = P1 + b * [cos(theta_rad), sin(theta_rad)];
% 
%     grid = make_simple_line(N, P1, P2);
% 
%     % Display grid with inverted Y-axis
%     imagesc(grid);  % Flip Y-axis for Cartesian view
%     colormap([1 1 1; 0 0 0]);
%     axis equal tight;
%     % Flip Y axis to match grid orientation
%     set(gca, 'YDir', 'normal');
%     xlabel('X');
%     ylabel('Y');
%     title(['Line at ', num2str(th), '°']);
%     pause(1);  % Pause to view each line
% end



%% Part 1 - get CB layers for A and B1

% Suppose Maximum radius of boundary point moving is R=sqrt(65)

delta_grid = 1;
% Define total grid, where physical grid is 32x32:
GridR = 32/delta_grid; % Dimension of cells in grid, 32, 64 = 32/desired delta
R = sqrt(65);
beta = 1;

N = ceil(2*pi*R/ (delta_grid) ) * beta; % Round-Up * 1/6
delta_th = 2*pi/N;

% N = ceil(2*pi/asin(delta_grid/ (15*R) ));

% ---------------------------FORCED N
% N = 32;

N = 70;

if N == 32
    chosen_i = [1,2,8, 16, 32];
else
    % chosen_i = [1,2, ceil(N/3), ceil(N/2), N];
    % chosen_i = [1,2,8,14, 16, 32,N-1,N];
    chosen_i = [1,2];
    % chosen_i = [1:1:N];
end

thetas = 0:2*pi/N:2*pi-2*pi/N;

% Robot A inital placement and relative vectors
dA = [4;24];
a1 = [4;24] - dA;
a2 = [12;24] - dA;
a3 = [12;25] - dA;
a4 = [4;25] - dA;


% Obstacle B1 vertices points
b1_B1 = [0;18];
b2_B1 = [10;18];
b3_B1 = [10;19];
b4_B1 = [0;19];

A_pts = [a1, a2, a3, a4];
nA = size(A_pts,2);
B1_pts = [b1_B1, b2_B1, b3_B1, b4_B1];

% [max_vert_P1,Layers_vert_P1]  = plot_calc_CB(A_pts, B1_pts,N,dA);

%% Part 2 - get CB for all obstacles in room:

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
CB_theta0_Poly_list = cell(nObst, N);

for i = 1:nObst
    [max_vert_Pi,Layers_vert_Pi]  = just_calc_CB(A_pts, Obstacles(:,:,i),N);
    for j = 1:N
        CB_t0 = Layers_vert_Pi(:,1:max_vert_Pi(j),j);
        x = CB_t0(1,:);y = CB_t0(2,:);
        CB_theta0_Poly_list{i,j} = polyshape(x,y);
    end
end

%% Now, merge the CB_theta0 in each layer:

% Define hole (inner rectangle)
hole = [1 1; 31 1; 31 29; 1 29];
th = 10; % Rectangle Thickness

% Compute outer rectangle based on thickness
x_min = min(hole(:,1)) - th;
x_max = max(hole(:,1)) + th;
y_min = min(hole(:,2)) - th;
y_max = max(hole(:,2)) + th;

% Define outer rectangle (larger than the hole)
outer = [x_min y_min;
         x_max y_min;
         x_max y_max;
         x_min y_max];

% Split X and Y into separate vectors for outer and hole
x_cells = {outer(:,1), hole(:,1)};
y_cells = {outer(:,2), hole(:,2)};

% Create polyshape with a hole
Bound = polyshape(x_cells, y_cells);

% Intilize CB_tot and its max_vert
CB_tot_perLayer = cell(nObst, N);

for i = 1:N
    CB_theta0_P_list_iLayer = cell(nObst + 1,1);
    for k = 1:nObst
        CB_theta0_P_list_iLayer{k} = CB_theta0_Poly_list{k,i};
    end
    % For each layer, add giant rect with hole to show wall:
    CB_theta0_P_list_iLayer{nObst + 1} = Bound;
    
    union_iLayer = union_polygons(CB_theta0_P_list_iLayer);
    
    Num_JointCB_th0 = size(union_iLayer,1);
    for j = 1:Num_JointCB_th0
        CB_tot_perLayer{j,i} = union_iLayer{j};
    end 

end

% Round some points if possible:
for i = 1:N
    CB_tot_layi = CB_tot_perLayer{1,i};
    Vertices = CB_tot_layi.Vertices;  
    for k = 1:size(Vertices,1)
        if not(isnan(Vertices(k,1))) 
            A = Vertices(k,1);
            B = Vertices(k,2);

            % Round very-integery numbers:
            if abs(A - round(A)) < 1e-8
                Vertices(k,1) = round(A);
            end
            if abs(B - round(B)) < 1e-8
                Vertices(k,2) = round(B);
            end
        end
    end
    CB_tot_layi.Vertices = Vertices;
    CB_tot_perLayer{1,i} = CB_tot_layi;
end

color_matrix = gen_col(nObst);

% Print CB for the layers:
for i = 1:N
    if ismember(i, chosen_i)
        figure();
        PrintCB;
        title(sprintf('CB for Layer %d', i));


        exportgraphics(gcf, sprintf('Robot_CB_tot_%d.png', i),...
            'Resolution', 300);
    end
end


%% Part 3 - Make a GridRxGridRxN grid

% 0 - for free-space and 1 - for CB

if Method == 1
    grid_Layers = ...
     GetDiscreteMap_Method1(CB_tot_perLayer, GridR, N,delta_grid,chosen_i);
else
    grid_Layers = ...
     GetDiscreteMap_Method2(CB_tot_perLayer, GridR, N,delta_grid,chosen_i,Free_pixel_check);
end

%% Print CB AND Discrete maps for the layers:
if Method == 2
    for i = 1:N
        if ismember(i, chosen_i)
            figure();
            PrintCB;
            grid_Layer_i = grid_Layers(:,:,i);
            % printGridLayer(grid_Layer_i, delta_grid,i,N,GridR)
            % Draw filled blocks for grid_Layer_i == 1 or 2
            for ii = 1:GridR
                for j = 1:GridR
                    x = (j-1) * delta_grid;
                    y = (ii-1) * delta_grid;
        
                    if grid_Layer_i(ii,j) == 0
                        % Draw black square
                        fill([x x+delta_grid x+delta_grid x], ...
                             [y y y+delta_grid y+delta_grid], ...
                             'g');
                    end
                end
            end
            title(sprintf('CB for Layer %d', i));
    
    
            % exportgraphics(gcf, sprintf('Robot_CB_And_Discrete_%d.png', i),...
            %     'Resolution', 300);
        end
    end
end


%% Make a GridRxGridRxN grid, ONLY BOUNDARY

if Method == 1

    grid_Layers_bound = grid_Layers;

else
    % 0 - for free-space and 1 - for INNER of CB (just boundary)

    % Define total grid, where physical grid is 32x32:
    grid_Layers_bound = zeros(GridR,GridR,N); % 1's for CB hole bounderies
    
    
    for i = 1:N
    
        A = grid_Layers(:,:,i);
        grid_Layers_bound(:,:,i) = mark_touching_ones_with_diagonals(A);
        grid_Layer_i = grid_Layers_bound(:,:,i);
        if ismember(i, chosen_i)
            % Create a new figure
            figure;
            printGridLayer(grid_Layer_i, delta_grid,i,N,GridR)
            % Export image
            exportgraphics(gcf, sprintf('Robot_CB_tot_0_1_Boundary_%d.png', i), 'Resolution', 300);
        end  
    
    end
end

%% Make GridRxGridRxN grid, Free-space, boundary AND outside marked w. 2's

% First, check that no c-obstacle edge moves more than 1 cell
% bet. two neighboring layers:

% 1's for CB inner hole bounderis, 2's for outer holes

if Method == 1
    grid_Layers_w2 = fillOutsideZeroRegions(grid_Layers);
    for i = 1:N
        if ismember(i, chosen_i)
            figure();
            grid_Layers_w2_i = grid_Layers_w2(:,:,i);
            printGridLayer(grid_Layers_w2_i, delta_grid,i,N,GridR)
            % exportgraphics(gcf, sprintf('Robot_CB_tot_0_1_2_%d.png', i), 'Resolution', 300);  
        end  
    end
else
    grid_Layers_w2 = zeros(GridR,GridR,N);
    
    for i = 1:N
        
        for row = 1:GridR
            for col = 1:GridR
                if grid_Layers(row,col,i) ~= 0 &&...
                        grid_Layers_bound(row,col,i) ~=1
                    grid_Layers_w2(row,col,i) = 2;
                elseif grid_Layers(row,col,i) == 0
                    grid_Layers_w2(row,col,i) = 0;
                else
                    grid_Layers_w2(row,col,i) = 1;
                end
            end
        end
    
        grid_Layers_w2_i = grid_Layers_w2(:,:,i);
        if ismember(i, chosen_i)
            % Create a new figure
            figure;
            printGridLayer(grid_Layers_w2_i, delta_grid,i,N,GridR)
            % Export image
            % exportgraphics(gcf, sprintf('Robot_CB_tot_0_1_2_%d.png', i), 'Resolution', 300);
        end  
    
    end
end

%% Check if resolution on N is good enough, for not using marked OUTSIDE:

% Initilize isolated for checking if N is good enought:
isolated = true;

for k = 1:N
    currentLayer = grid_Layers_w2(:,:,k);
    % Initialize top and bottom layers
    if k > 1
        belowLayer = grid_Layers_w2(:,:,k-1);
    else
        belowLayer = grid_Layers_w2(:,:,N);  % treat out-of-bounds as 2
    end

    if k < N
        aboveLayer = grid_Layers_w2(:,:,k+1);
    else
        aboveLayer = grid_Layers_w2(:,:,1);  % treat out-of-bounds as 2
    end

    % Condition: current is 0, and both neighbors are NOT 2
    for row = 1:GridR
        for col = 1:GridR
            if currentLayer(row,col) == 0
                isolated_rowcol = aboveLayer(row,col) ~=2 &...
                              belowLayer(row,col) ~=2;
                if isolated_rowcol ~= 1
                    X = ['Lay:',num2str(k),' x,y:',...
                        num2str((col-1)*delta_grid),',',num2str((row-1)*delta_grid)];
                    disp(X)
                end
                isolated = isolated & isolated_rowcol;
            end
        end
    end
end

X = ['Is the N big enough? ',num2str(isolated)];
disp(X)

%% Store free-space nodes

% marks free-space as 0's, rest is 1's
free_sp_layers = cell(1,N);

% Saves collums of [row,col,lyr] of free-spaces
free_space_nodes = [];
% Saves collums of [x,y,theta] of free-spaces
free_space_nodes_xyth = [];

% Also, get S and T in node corrdinates:
dT = [4;8];
S_xyth = [dA;0];
T_xyth = [dT;0];
S_Cor = [round(dA(2)/delta_grid)+1;round(dA(1)/delta_grid)+1;1];
T_cor = [round(dT(2)/delta_grid)+1;round(dT(1)/delta_grid)+1;1];
S = -1;
T = -1;


% First, store all free-space without connections
for k = 1:N
    currentLayer = grid_Layers_w2(:,:,k);
    current_free_sp = ones(GridR); 
    % Matrix - mark 0 as Free-space, rest is 1

    for row = 1:GridR
        for col = 1:GridR
            if currentLayer(row,col) == 0 % Free-space
                current_free_sp(row,col) = 0;
                free_space_nodes = [free_space_nodes,[row;col;k]];

                if isequal(S_Cor,[row;col;k])
                    S = size(free_space_nodes,2);
                end
                if isequal(T_cor,[row;col;k])
                    T = size(free_space_nodes,2);
                end

                % Free-space is middle of pixel in XY grid
                if Free_pixel_check == 1 % Point of pixel in bottom-left
                    free_space_nodes_xyth = ...
                    [free_space_nodes_xyth,...
                    [(col-1)*delta_grid;...
                    (row-1)*delta_grid;delta_th*(k-1)]];
                else % Point of pixel in middle
                    free_space_nodes_xyth = ...
                    [free_space_nodes_xyth,...
                    [(col-1)*delta_grid+0.5*delta_grid;...
                    (row-1)*delta_grid+0.5*delta_grid;delta_th*(k-1)]];
                end

            end
        end
    end
    
    free_sp_layers{k} = current_free_sp;
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

    % Fix layers to be cyclic in 1->N
    bottom_lyr = lay_cur-1;
    top_lyr = lay_cur+1;
    if lay_cur == 1
        bottom_lyr = N;
    end
    if lay_cur == N
        top_lyr = 1;
    end

    % In general - 6 neighbors in 3D grid, will delete some l8tr
    neighbors_idx = [[row_cur-1;col_cur;lay_cur],[row_cur;col_cur-1;lay_cur],...
        [row_cur+1;col_cur;lay_cur],[row_cur;col_cur+1;lay_cur],...
        [row_cur;col_cur;bottom_lyr],[row_cur;col_cur;top_lyr]];
    idx_delet = [0,0,0,0,0,0]; % 0-keep, 1-delete
    

    % Fix indecies to be inside matrix 
    % & check neighbor to be in free-space
    for q = 1:6
        rowMat_q = neighbors_idx(1,q);
        colMat_q = neighbors_idx(2,q);
        lay_cur_q = neighbors_idx(3,q);
        
        % Remove neighbors that have non-relevant indices
        if rowMat_q<1 || rowMat_q>GridR ...
               || colMat_q<1 || colMat_q>GridR
            idx_delet(q) = 1;
        % Check if the neighbor is even in free-space:
        elseif free_sp_layers{1,lay_cur_q}(rowMat_q,colMat_q) == 1
            idx_delet(q) = 1;
        end
        
    end

    % Keep specific neighbors:
    neighbors_idx_keep = [];
    for k = 1:6
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
view(3);
title('3D Points from Path Array');
axis equal;

exportgraphics(gcf, 'Robot_Path_xylayer.png', 'Resolution', 300);

%% Print 3D closed nodes

% Plot
index_elipses = [10,30,size(C,2)]; % put index > 1
% Dist in xy-1, in th-0.1232
l_path_lst = [3.1232*2,3.3696*2,l_path]; % Nav 1
l_path_lst = [4.1232*2,5.616*2,l_path]; % Nav 2
% increase small prices, mathmaticly bounds but too small for graphing

for i = 1:length(index_elipses)
    
    indx = index_elipses(i);
    FirstLocX = free_space_nodes_xyth(1,C(1,1));
    FirstLocY = free_space_nodes_xyth(2,C(1,1));
    FirstLocZ = free_space_nodes_xyth(3,C(1,1));
    OtherLocX = free_space_nodes_xyth(1,C(1,indx));
    OtherLocY = free_space_nodes_xyth(2,C(1,indx));
    OtherLocZ = free_space_nodes_xyth(3,C(1,indx));

    l_path_curr = l_path_lst(i);

    figure;
    X_path = free_space_nodes_xyth(1,C(1,1:indx));
    Y_path = free_space_nodes_xyth(2,C(1,1:indx));
    Z_path = free_space_nodes_xyth(3,C(1,1:indx));

    % Remove cyclic of theta:
    if FirstLocZ > pi
        FirstLocZ = FirstLocZ - 2*pi;
    end
    if OtherLocZ > pi
        OtherLocZ = OtherLocZ - 2*pi;
    end
    for q = 1:length(Z_path)
        if Z_path(i) > pi
            Z_path(i) = Z_path(i) - 2*pi;
        end
    end

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
    title(sprintf('Robot C Elipsoids for element i=%d',indx));
    axis equal;
    
    % Add NORM 1 elipsoid:
    % Create 3D grid
    if i ~= length(index_elipses)
        [x, y, z] = meshgrid(linspace(1, 15, 50), ...
                         linspace(10, 29, 50), ...
                         linspace(0, 0.3*pi, 50));
    else
        [x, y, z] = meshgrid(linspace(-100, 100, 50), ...
                         linspace(-100, 100, 50), ...
                         linspace(0, 1/12*pi, 50));
    end
    
    
    % Compute distances to the foci
    d1 = abs(x - FirstLocX) + abs(y - FirstLocY) + abs(z - FirstLocZ);
    d2 = abs(x - OtherLocX) + abs(y - OtherLocY) + abs(z - OtherLocZ);
    
    % Isosurface where the sum of distances equals 2a
    v = d1 + d2;
    h = isosurface(x, y, z, v, l_path_curr);
    p = patch(h);
    set(p, ...
        'FaceColor', [0.5 0.5 0.5], ...   % gray color (RGB)
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.3);               % transparency (0 = transparent, 1 = opaque)
    
    hold on;
    plot3(S_xyth(1), S_xyth(2), S_xyth(3), 'bo', 'MarkerSize', 8, 'DisplayName','S')
    plot3(T_xyth(1), T_xyth(2), T_xyth(3), 'bo', 'MarkerSize', 8, 'DisplayName','T')
    
    plot3(FirstLocX, FirstLocY, FirstLocZ, 'ro', 'MarkerSize', 8, 'DisplayName','S')
    plot3(OtherLocX, OtherLocY, OtherLocZ, 'ro', 'MarkerSize', 8, 'DisplayName','T')
    
    xlim([0,32]);
    ylim([0,30]);

    exportgraphics(gcf, sprintf('Robot_C_xylayer_%d.png',indx), 'Resolution', 300);
end

%% Print robot doing the path:

% Parameters
T = 10;  % total video time in seconds
if Free_pixel_check == 2
    num_frames = length(path) + 2; 
else
    num_frames = length(path); 
end

% number of steps in the robot's path (including begging and end)
dt = T / num_frames;  % time step to match video duration

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
v = VideoWriter('robot_path_silhouette.mp4', 'MPEG-4');  % <-- MP4 output
v.FrameRate = 1 / dt;
open(v);

% Preallocate patch handles for silhouette polygons to improve speed
silhouette_patches = gobjects(num_frames,1);

for i = 1:num_frames
    % Get current robot position and orientation
    if Free_pixel_check == 2
        if i ~= 1 && i ~= num_frames
            x = free_space_nodes_xyth(1, path(i-1));
            y = free_space_nodes_xyth(2, path(i-1));
            layer = free_space_nodes(3, path(i-1));
            theta = thetas(layer);
        elseif i == 1 % S_Cor
            x = dA(1);
            y = dA(2);
            layer = 1;
            theta = thetas(layer);
            
        else  % T_Cor
            x = dT(1);
            y = dT(2);
            layer = 1;
            theta = thetas(layer);
        end
    else
        x = free_space_nodes_xyth(1, path(i));
        y = free_space_nodes_xyth(2, path(i));
        layer = free_space_nodes(3, path(i));
        theta = thetas(layer);
    end
    
    

    % Rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

    % Rotate and translate robot points
    A_pts_rot = R * A_pts;
    A_trans = A_pts_rot + [x; y];

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
exportgraphics(gcf, 'Robot_Silhouette.png', 'Resolution', 300);