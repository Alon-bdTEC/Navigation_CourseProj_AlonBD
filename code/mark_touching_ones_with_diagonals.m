function B = mark_touching_ones_with_diagonals(A)
%MARK_TOUCHING_ONES_WITH_DIAGONALS Detects 1s touching 0s and adds diagonals
%   B(i,j) == 1 if (i,j) is:
%     - a 1 in A that touches at least one 0 (4-connectivity)
%     - or a new 1 added diagonally between two such neighbors of a 0

    [rowN, colN] = size(A);
    B = zeros(rowN, colN);  % result

    % First pass: Mark 1's that are 4-connected to a 0
    for y = 1:rowN
        for x = 1:colN
            if A(y,x) == 0
                % right
                if x+1 <= colN && A(y, x+1) == 1
                    B(y, x+1) = 1;
                end
                % down
                if y+1 <= rowN && A(y+1, x) == 1
                    B(y+1, x) = 1;
                end
                % left
                if x-1 >= 1 && A(y, x-1) == 1
                    B(y, x-1) = 1;
                end
                % up
                if y-1 >= 1 && A(y-1, x) == 1
                    B(y-1, x) = 1;
                end
            end
        end
    end

    % Second pass: Add diagonals where two orthogonal 1's touch a 0
    for y = 1:rowN
        for x = 1:colN
            if A(y,x) == 0
                % right & down
                if x+1 <= colN && y+1 <= rowN && ...
                   A(y, x+1) == 1 && A(y+1, x) == 1
                    B(y+1, x+1) = 1;
                end
                % right & up
                if x+1 <= colN && y-1 >= 1 && ...
                   A(y, x+1) == 1 && A(y-1, x) == 1
                    B(y-1, x+1) = 1;
                end
                % left & down
                if x-1 >= 1 && y+1 <= rowN && ...
                   A(y, x-1) == 1 && A(y+1, x) == 1
                    B(y+1, x-1) = 1;
                end
                % left & up
                if x-1 >= 1 && y-1 >= 1 && ...
                   A(y, x-1) == 1 && A(y-1, x) == 1
                    B(y-1, x-1) = 1;
                end
            end
        end
    end
end
