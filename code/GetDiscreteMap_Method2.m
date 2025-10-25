function grid_Layers = GetDiscreteMap_Method2(CB_tot_perLayer, GridR, N,delta_grid,chosen_i,Free_pixel_check)
    % 0 - for free-space and 1 - for INNER of CB
    
    % Define total grid, where physical grid is 32x32:
    grid_Layers = zeros(GridR,GridR,N); % 1's for CB itself
    
    % Generate discrete grid when CB is intersecting the pixels (touching
    % dosent count!)
    
    for i = 1:N
    
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
    
        for xi = 0:GridR-1
            for yi = 0:GridR-1

                isCenterFree = false;
    
                for k = 1:length(sectionStarts) % Run though CB Inner-FreeSpaces
                    points_x = boundary_CB_x(sectionStarts(k):sectionEnds(k), 1);
                    points_y = boundary_CB_y(sectionStarts(k):sectionEnds(k), 1);
                    
                    if Free_pixel_check == 1
                        [inFree,~] = inpolygon(xi,yi,points_x,points_y);
                    else
                        [inFree,~] = inpolygon(xi+0.5,yi+0.5,points_x,points_y);
                    end
                    
                    
                    isCenterFree = isCenterFree || inFree;
    
                end
                
    
                % If the cell didnt get marked AND it overlaps (with area) CB
                if not(isCenterFree)
                    grid_Layers(yi+1,xi+1,i) = 1;
                end
            end
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