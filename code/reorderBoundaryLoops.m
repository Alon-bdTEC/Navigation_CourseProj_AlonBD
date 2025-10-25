function [x_out, y_out] = reorderBoundaryLoops(x, y)
% Moves a manually-specified loop (by known coordinates) to the end

    x = x(:); y = y(:);
    nan_idx = find(isnan(x) | isnan(y));
    splits = [0; nan_idx; numel(x)+1];

    loops = cell(numel(splits)-1, 1);
    for i = 1:numel(loops)
        idx = (splits(i)+1):(splits(i+1)-1);
        loops{i} = [x(idx), y(idx)];
    end

    % Define the 5 constant coordinates
    ref_coords = [
        -9, -9;
        -9, 39;
        41, 39;
        41, -9;
        -9, -9
    ];

    % Count-based match (each ref point must be found the correct number of times)
    outer_idx = [];
    for i = 1:numel(loops)
        loop = loops{i};
        found = 0;
        for j = 1:size(ref_coords,1)
            % Count how many times ref_coords(j,:) appears in loop
            matches = all(bsxfun(@eq, loop, ref_coords(j,:)), 2);
            if sum(matches) >= 1
                found = found + 1;
            end
        end

        if found == size(ref_coords,1)
            outer_idx = i;
            break;
        end
    end

    if isempty(outer_idx)
        error('Could not find the manual outer loop');
    end

    % Reorder loops: holes first, outer last
    hole_idx = setdiff(1:numel(loops), outer_idx);
    all_loops = [loops(hole_idx); loops(outer_idx)];

    % Recombine with NaNs
    xy = [];
    for i = 1:numel(all_loops)
        xy = [xy; NaN NaN; all_loops{i}];
    end
    xy(1,:) = [];  % remove leading NaN

    x_out = xy(:,1);
    y_out = xy(:,2);
end
