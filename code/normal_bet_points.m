function n = normal_bet_points(p1,p2)
% normal_bet_points: Returns Normal between two CCW points in polygon
%   Input:
%       p1,p2 - two points in 2D
%   Output:
%       n - unit vector, noraml to polygon segment between p1 and p2 CCW
%

    r_2_rel_1 = p2-p1;

    % 90 deg rotate of relative distance of CCW points:
    theta = -deg2rad(90);
    n = [cos(theta) -sin(theta);sin(theta) cos(theta)]*r_2_rel_1;
    n = n/norm(n);

end
