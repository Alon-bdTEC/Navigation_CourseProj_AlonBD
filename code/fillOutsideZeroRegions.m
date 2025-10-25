function markedMatrix = fillOutsideZeroRegions(matrix)
    % Get matrix size
    [rows, cols] = size(matrix);
    
    % Create a visited matrix
    visited = false(rows, cols);
    
    % Create a queue for flood fill
    queue = java.util.LinkedList();
    
    % Add border 0's to the queue
    for r = 1:rows
        for c = [1, cols]
            if matrix(r, c) == 0 && ~visited(r, c)
                queue.add([r, c]);
                visited(r, c) = true;
            end
        end
    end
    for c = 1:cols
        for r = [1, rows]
            if matrix(r, c) == 0 && ~visited(r, c)
                queue.add([r, c]);
                visited(r, c) = true;
            end
        end
    end
    
    % Directions: up, down, left, right
    directions = [-1 0; 1 0; 0 -1; 0 1];
    
    % Flood fill from border 0's
    while ~queue.isEmpty()
        pos = queue.remove();
        r = pos(1);
        c = pos(2);
        matrix(r, c) = 2;
        
        for d = 1:4
            nr = r + directions(d, 1);
            nc = c + directions(d, 2);
            if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                if matrix(nr, nc) == 0 && ~visited(nr, nc)
                    queue.add([nr, nc]);
                    visited(nr, nc) = true;
                end
            end
        end
    end
    
    markedMatrix = matrix;
end
