function intersects = segments_int(v1, v2)
    % segments_int: checks if two angular segments intersect on [0, 2pi)
    % Inputs: v1 = [a1, a2], v2 = [b1, b2], CCW sections from angle a1 to angle a2 and angle b1 to angle b2
    % Output: intersects = true if the two angular segments intersect (CCW)
    
    % Normalize angles to [0, 2*pi)
    v1 = mod(v1, 2*pi);
    v2 = mod(v2, 2*pi);
    
    % Helper to check if angle x is within the CCW arc from a to b
    in_arc = @(x, a, b) ...
        (a < b && x >= a && x <= b) || ...
        (a > b && (x >= a || x <= b));

    % Check if either endpoint of v2 lies inside v1's arc, or vice versa
    intersects = in_arc(v2(1), v1(1), v1(2)) || ...
                 in_arc(v2(2), v1(1), v1(2)) || ...
                 in_arc(v1(1), v2(1), v2(2)) || ...
                 in_arc(v1(2), v2(1), v2(2));
end
