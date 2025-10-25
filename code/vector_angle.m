function theta = vector_angle(v)
%VECTOR_ANGLE Returns the angle of a 2D vector in radians within [-pi, pi]
%   Input:
%       v - a 2-element column or row vector [x; y] or [x, y]
%   Output:
%       theta - angle in radians in the range [0, 2*pi)

    % Validate input
    if numel(v) ~= 2
        error('Error: Input must be a 2-element vector [x; y]');
    end

    % Extract components
    x = v(1);
    y = v(2);

    % Compute angle using atan2
    theta = atan2(y, x);
    % Normalize all angles to [0, 2*pi)
    theta = mod(theta + 2*pi, 2*pi);
end
