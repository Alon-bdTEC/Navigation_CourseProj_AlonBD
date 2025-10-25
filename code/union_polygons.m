function union_Plist = union_polygons(P_list)
    % Input:
    %   P_list: cell array {np×1}, each cell is a polyshape
    % Output:
    %   union_Plist: cell array {nF×1}, each is a merged polyshape

    np = length(P_list);
    active = true(1, np);  % Track which polygons are still active

    changed = true;
    while changed
        changed = false;
        for i = 1:np
            if ~active(i), continue; end
            poly_i = P_list{i};

            for j = i+1:np
                if ~active(j), continue; end
                poly_j = P_list{j};

                merged_poly = intersect_two_polygons(poly_i, poly_j);

                if isempty(merged_poly)
                    continue;
                end

                % Update polygon i with merged result
                P_list{i} = merged_poly;

                % Mark polygon j as inactive
                active(j) = false;

                changed = true;
                break;  % Restart from outer loop
            end
            if changed
                break;
            end
        end
    end

    % Collect active (merged) polygons into output list
    nF = sum(active);
    union_Plist = cell(nF, 1);

    idx = 1;
    for j = 1:np
        if active(j)
            union_Plist{idx} = P_list{j};
            idx = idx + 1;
        end
    end
end
