function union_poly = intersect_two_polygons(P1, P2)
    % Inputs:
    %   P1, P2: polyshape objects
    % Output:
    %   union_poly: polyshape object representing the union,
    %               or [] if disjoint

    % Check for intersection (skip disjoint cases)
    if isempty(intersect(P1, P2).Vertices)
        union_poly = [];
        return;
    end

    % Return union of the two polygons
    union_poly = union(P1, P2);
end
