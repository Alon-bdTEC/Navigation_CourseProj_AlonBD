function grid_Layers = GetDiscreteMap_Method1(CB_tot_perLayer, GridR, N,delta_grid,chosen_i)
    % 0 - for free-space and 1 - for CB
    
    % Define total grid, where physical grid is 32x32:
    grid_Layers = zeros(GridR,GridR,N); % 1's for CB itself
    
    for i = 1:N
        i
    
        CB_Li = CB_tot_perLayer{1,i};
        % CB_Li = getOrderedPolyshape(CB_Li,known_outer);

        [boundary_CB_x,boundary_CB_y] = boundary(CB_Li);
    
        [boundary_CB_x, boundary_CB_y] =...
            reorderBoundaryLoops(boundary_CB_x, boundary_CB_y);
    
        % Remove OUTER LOOP + its null:
        boundary_CB_x = boundary_CB_x(1:end-6);
        boundary_CB_y = boundary_CB_y(1:end-6);
        
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
    
        for k = 1:length(sectionStarts) % Run though CB Inner-FreeSpaces
            points = [boundary_CB_x(sectionStarts(k):sectionEnds(k), 1),...
                boundary_CB_y(sectionStarts(k):sectionEnds(k), 1)];
            np = size(points,1);
    
            % Now loop through each edge in this section:
            for j = 1:np-1
                % Get point of edge (normalized in grid)
                P1 = points(j, :) ./delta_grid;
                P2 = points(j+1, :) ./delta_grid;
                
                grid_w_P1P2 = make_simple_line(GridR, P1, P2);
    
                grid_Layers(:,:,i) = grid_Layers(:,:,i) | grid_w_P1P2;
            end
        
            % Close the loop: first and last points in boundary
            % (normalized in grid)
            P1 = points(np, :) ./delta_grid;
            P2 = points(1, :) ./delta_grid;
            
            grid_w_P1P2 = make_simple_line(GridR, P1, P2);
    
            grid_Layers(:,:,i) = grid_Layers(:,:,i) | grid_w_P1P2;
        end
    
        grid_Layer_i = grid_Layers(:,:,i);
        if ismember(i, chosen_i)
            % Create a new figure
            figure;
            printGridLayer(grid_Layer_i, delta_grid,i,N,GridR)
            % Export image
            exportgraphics(gcf, sprintf('Robot_CB_tot_0_1_%d.png', i), 'Resolution', 300);
        end  
    
    end
end