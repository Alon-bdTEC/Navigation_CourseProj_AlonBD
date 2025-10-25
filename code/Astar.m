function [path,C,prices] = Astar(graph, vertss, S, T)
    % O includes collums of [point;lb;dist;back_point (only one)]
    O = [S;0; norm(vertss(:,T)-vertss(:,S)); 0];
    % C has collumf of [points;back_point of this point]
    C = [];
    
    targetClosed = false;
    while not(isempty(O)) && not(targetClosed)
        % C takes [points;back_point of this point] from O
        C = [C, O([1,4],1)];
        xbest = O(1,1);lb_best = O(2,1);

        if O(1,1) == T % If the first collum in O is the Target
            targetClosed = true;
        end

        O(:,1)=[];
        % x_best is the first collum in O (need to sort O every time)
        nodeConections = [];
        nodeConections = graph{xbest}; % Collums of [neighbor index;l_b]
        for i =1:size(nodeConections,2)
            neighbor_index = nodeConections(1,i);
            neighbor_l_b = nodeConections(2,i);
            norm_wtheta = normVectorZTheta(vertss(:,neighbor_index),vertss(:,T));

            if not(ismember(neighbor_index,C(1,:)))
                % t_x = lb_best + nodeConections(neighbor_index) +...
                %       norm(vertss(:,neighbor_index)-vertss(:,T));
                
                if not(ismember(neighbor_index,O(1,:)))
                    % If its not a member of O, add it and sort
                    O = [O,[neighbor_index; lb_best+neighbor_l_b;...
                        norm_wtheta; xbest]];
                    % Compute the sum of rows 2 and 3 for each column
                    sums = O(2,:) + O(3,:);
                    
                    % Get the sorting indices based on the computed sums
                    [~, idx] = sort(sums);
                    
                    % Sort the columns of O using the indices
                    O_sorted = O(:, idx);

                    O = O_sorted;
                else
                    % Get index where this neighbor is in O:
                    indx_O = find(O(1,:)==neighbor_index,1);
                    lb_O = O(2,indx_O);
                    % Positive, means replace in O
                    diff = (lb_O) - (lb_best + neighbor_l_b); 
                    if diff >= 0
                        O(:,indx_O) = [neighbor_index;lb_best+neighbor_l_b;...
                           norm_wtheta ;xbest];
                        
                    end
                end
            end
        end
    end

    path = [];prices = [];
    if targetClosed
        path = findPathFromC(C, S);
        prices = findPrice(C, path,graph);
    end
end

function path = findPathFromC(C, S)
    path = [C(1,end)];
    while path(1) ~= S
        path = [C(2,C(1,:)==path(1)), path];
    end
end

function prices = findPrice(C, path,graph)
    prices = [];
    nPath = length(path);
    for i = 0:(nPath-1)
        % Find instance of path node in C (in reverse order)
        % Eeach closed node, from T, gets you to S using the back-pointers
        pathNode = path(nPath-i);
        pathNodeNext = pathNode;
        for q = 0:size(C,2)-1
            if C(1,size(C,2)-q) == pathNode
                pathNodeNext = C(2,size(C,2)-q);
                break;
            end
        end
        
        neighbors_path_node = graph{pathNode};

        for j = 1:size(neighbors_path_node,2)
            if neighbors_path_node(1,j) == pathNodeNext
                price = neighbors_path_node(2,j);
                break;
            end
        end
        prices = [price, prices];
    end
end

