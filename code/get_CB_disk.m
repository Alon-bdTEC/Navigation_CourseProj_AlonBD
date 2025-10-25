function polyshape_vert_w_circles = get_CB_disk(D, B_pts)
    % get_CB_disk: finds CB for disk and convex obstacle
    %
    % Inputs:
    %   D - Robot disk diameter.
    %
    %   B_pts = [b1, b2,..] - 2xnB, points of convex B in world frame
    %
    %   dA - position of A's center in start
    %
    %   CCW counts of robot A and obstalce B vertices seperatly
    %
    %   Output: Calculates and plots vertices of CBs and prints it.
    %
    %   polyshape_vert_w_circles - all vertices of CB
    %
    
    % -----------Defining Parameters-----------
    % Define nB:
    nB = size(B_pts,2);

    R = D/2;

    % -----------Firstly, rotate robot A in angle theta-----------
    vert = [];
    vert_w_circles = [];
    % Disk CB of conves polygon & disk:
    % first is the D/2 extrude of each edge
    % than, circles between the lines with center in the obstacle
    % vertex

    % First, move all edges by R
    for i = 1:nB

        if i ~= nB
            bpt_1 = B_pts(:,i);
            bpt_2 = B_pts(:,i+1);
        else
            bpt_1 = B_pts(:,i);
            bpt_2 = B_pts(:,1);
        end

        n = normal_bet_points(bpt_1,bpt_2);

        displacement_vector = n*(R);

        bpt_1 = bpt_1 + displacement_vector;
        bpt_2 = bpt_2 + displacement_vector;
        
        vert = [vert, bpt_1, bpt_2];

    end

    % Now, add the circles: n_vert = size(vert,2) is always even
    % 2-3, 4-5,..., (n_vert-1)-n_vert, n_vert-1
    
    n_vert = size(vert,2);
    j = 1;
    for i = 2:2:n_vert
        j = j + 1;
        
        if j <= nB
            origin = B_pts(:,j);
        else
            origin = B_pts(:,1);
        end
        

        if i ~= n_vert % while its not the last two pts
            vert_pt_1 = vert(:,i);
            vert_pt_2 = vert(:,i+1);
        else
            vert_pt_1 = vert(:,i);
            vert_pt_2 = vert(:,1);
        end
        
        O = origin;
        P1 = vert_pt_1;
        P2 = vert_pt_2;
        
        % Compute angles of P1 and P2 with respect to origin
        theta1 = atan2(P1(2) - O(2), P1(1) - O(1));
        theta2 = atan2(P2(2) - O(2), P2(1) - O(1));
        
        % Ensure counterclockwise direction
        if theta2 <= theta1
            theta2 = theta2 + 2*pi;
        end
        
        % Generate arc points
        nPoints = 100;
        theta = linspace(theta1, theta2, nPoints);
        xArc = O(1) + R * cos(theta);
        yArc = O(2) + R * sin(theta);

        CirclePts = [xArc;yArc];
        
        vert_w_circles = [vert_w_circles,CirclePts];
        
    end
    
    polyshape_vert_w_circles = ...
        polyshape(vert_w_circles(1,:),vert_w_circles(2,:));

end