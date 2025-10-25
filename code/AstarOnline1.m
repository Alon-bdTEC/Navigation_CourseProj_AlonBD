function [path,C,List_FromCurrToNewBest] = AstarOnline1(graph, vertss, S, T)
    % O includes collums of [point;lb;dist;back_point (only one)]
    O = [S;0; norm(vertss(:,T)-vertss(:,S)); 0];
    % C has collumf of [points;back_point of this point (only one)]
    C = [];
    
    % list in which the collums are the path from old the new x_best
    % in each itiration in the while loop
    List_FromCurrToNewBest = {};

    targetClosed = false;
    while not(isempty(O)) && not(targetClosed)
        
        if ~isempty(C)
            xbestPrev = C(1,end);
            back_points_Prev = C(2,end);
        end

        % C takes [points;back_point of this point] from O
        C = [C, O([1,4],1)];
        xbestNew = O(1,1);lb_best = O(2,1);
        % back_points_new = O(4,1);
        
        if O(1,1) == T % If the first collum in O is the Target
            targetClosed = true;
        end

        % Add path from current to new x_best
        if O(4,1) == 0 
            % both the previous and new x_best=S
            % FIRST ITIRATION
        elseif isempty(C) || back_points_Prev == 0 
            % ONLY the previous x_best=S - ONLY go to next x_best

            % From S to x_bestNew
            forward_pt_S_to_XbestNew = findBackPointer(C,xbestNew);

            List_FromCurrToNewBest{end+1} = forward_pt_S_to_XbestNew;
        else
            % need to go from x_bestPrev to S and than to x_bestNew

            % From x_bestPrev to S
            back_pt_S_to_XbestPrev = flipud(findBackPointer(C,xbestPrev));

            % From S to x_bestNew
            forward_pt_S_to_XbestNew = findBackPointer(C,xbestNew);
            
            total_trav = [back_pt_S_to_XbestPrev;forward_pt_S_to_XbestNew(2:end)];

            % inclued S and xbestNew
            List_FromCurrToNewBest{end+1} = total_trav;
        end

        O(:,1)=[];
        % x_best is the first collum in O (need to sort O every time)
        nodeConections = [];
        nodeConections = graph{xbestNew}; % Collums of [neighbor index;l_b]
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
                        norm_wtheta; xbestNew]];
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
                           norm_wtheta ;xbestNew];
                        
                    end
                end
            end
        end
    end

    path = [];
    if targetClosed
        path = findPathFromC(C, S);
    end
end

function path = findPathFromC(C, S)
    path = [C(1,end)];
    while path(1) ~= S
        path = [C(2,C(1,:)==path(1)), path];
    end
end

function back_pt_fromS_toX_best = findBackPointer(C,xbest)
    % This function gives the list of back-pointer from S to xbest
    % inclued S and xbest
    
    back_pt_fromS_toX_best = [];
    % find index of xbestNew in C
    indx_x_bestNew = -1;
    for indx_x_bestNew = 1:size(C,2)
        if C(1,indx_x_bestNew) == xbest
            break;
        end
    end
    
    indx_x_best_inINV = size(C,2) -...
        indx_x_bestNew + 1;
    C_inv = [fliplr(C(1,:));fliplr(C(2,:))];
    i = indx_x_best_inINV;
    x_loop = xbest;
    while C_inv(2,i) ~= 0 && i < size(C,2)
        if C_inv(1,i) == x_loop
            if x_loop == xbest
                back_pt_fromS_toX_best = [x_loop;back_pt_fromS_toX_best];
            end
            x_loop = C_inv(2,i);
            back_pt_fromS_toX_best = [x_loop;back_pt_fromS_toX_best];
        end 

        i = i + 1;
    end
end
