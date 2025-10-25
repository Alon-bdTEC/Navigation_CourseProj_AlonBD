function printGridLayer(grid_Layer_i, delta_grid, index_layer, N,GridR)

    domain_size = GridR * delta_grid;

    hold on;
    axis equal;
    axis([0 domain_size 0 domain_size]);
    if delta_grid == 1
        set(gca,'xtick',0:delta_grid:domain_size,'ytick',0:delta_grid:domain_size);
    else
        A = 1/delta_grid;
        set(gca,'xtick',0:delta_grid*A:domain_size,'ytick',0:delta_grid*A:domain_size);
    end
    
    grid on;
    box on;

    % Draw filled blocks for grid_Layer_i == 1 or 2
    for i = 1:GridR
        for j = 1:GridR
            x = (j-1) * delta_grid;
            y = (i-1) * delta_grid;

            if grid_Layer_i(i,j) == 1
                % Draw black square
                fill([x x+delta_grid x+delta_grid x], ...
                     [y y y+delta_grid y+delta_grid], ...
                     'k');
            elseif grid_Layer_i(i,j) == 2
                % Draw red square
                fill([x x+delta_grid x+delta_grid x], ...
                     [y y y+delta_grid y+delta_grid], ...
                     'r');
            end
        end
    end

    % Draw grid lines manually
    for i = 0:N
        % Horizontal lines
        plot([0 domain_size], [i*delta_grid i*delta_grid], 'k');
        % Vertical lines
        plot([i*delta_grid i*delta_grid], [0 domain_size], 'k');
    end

    xlabel('X');
    ylabel('Y');
    title(sprintf('CB in 0/1/2 grid, for Layer %d', index_layer));

end
