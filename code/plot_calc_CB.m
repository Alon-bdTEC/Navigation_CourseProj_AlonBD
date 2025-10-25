function [max_vert,Layers_vert] = plot_calc_CB(A_pts, B_pts,N,dA)
    % plot_calc_CB: finds CB for theta = 0 -> 2pi in N layers
    %
    % Inputs:
    %   A_pts = [a1, a2,..] - 2xnA, points of robot A relative
    %   to point a1 for theta = 0, Fa system is in a1.
    %
    %   B_pts = [b1, b2,..] - 2xnB, points of B in world frame
    %   
    %   N - Number of desired layers in CB
    %
    %   dA - position of A's a1 vertix in start
    %
    %   CCW counts of robot A and obstalce B vertices seperatly
    %
    % Output: Calculates and plots vertices of CB for thetas from 0->2pi
    %   in N steps. and print specific layers
    %
    %   Layers_vert is 2x(nA+nB)xN - all vertices of CB_theta_0 for
    %   thetas in range 0->2pi in N steps
    %
    %   max_vert is 1xN - saves number of vertices for each layer
    %
    
    % -----------Defining Parameters-----------
    % Define nA and nB:
    nA = size(A_pts,2);
    nB = size(B_pts,2);

    % -----------Firstly, rotate robot A in angle theta-----------
    Layers_vert = zeros(2,nA+nB,N);
    max_vert = zeros(1,N);
    thetas = 0:2*pi/N:2*pi-2*pi/N;
    A_pts_rot = zeros(2,nA,N);
    for i = 1:N
        theta = thetas(i);
        R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
        
        A_pts_rot(:,:,i) = A_pts;

        for k = 1:nA
            A_pts_rot(:,k,i) = R*A_pts_rot(:,k,i);
        end
        
        Layer_i = calc_CB_th_0(A_pts_rot(:,:,i), B_pts);
        max_vert(i) = size(Layer_i,2);
        Layers_vert(:,1:max_vert(i),i) = Layer_i;
    end

    % Plots specific angles:
    % For layer 1:
    plot_robot_and_obstacle(A_pts_rot(:,:,1), B_pts,dA);
    exportgraphics(gcf, 'RobotObstacle_1.png', 'Resolution', 300);

    plot_CB_single_theta(B_pts, Layers_vert(:,1:max_vert(1),1));
    exportgraphics(gcf, 'ObstacleAndCB_1.png', 'Resolution', 300);

    % For layer 2:
    plot_robot_and_obstacle(A_pts_rot(:,:,2), B_pts,dA);
    exportgraphics(gcf, 'RobotObstacle_2.png', 'Resolution', 300);

    plot_CB_single_theta(B_pts, Layers_vert(:,1:max_vert(2),2));
    exportgraphics(gcf, 'ObstacleAndCB_2.png', 'Resolution', 300);

    % For layer 8:
    plot_robot_and_obstacle(A_pts_rot(:,:,8), B_pts,dA);
    exportgraphics(gcf, 'RobotObstacle_8.png', 'Resolution', 300);

    plot_CB_single_theta(B_pts, Layers_vert(:,1:max_vert(8),8));
    exportgraphics(gcf, 'ObstacleAndCB_8.png', 'Resolution', 300);

    % For layer 16:
    plot_robot_and_obstacle(A_pts_rot(:,:,16), B_pts,dA);
    exportgraphics(gcf, 'RobotObstacle_16.png', 'Resolution', 300);

    plot_CB_single_theta(B_pts, Layers_vert(:,1:max_vert(16),16));
    exportgraphics(gcf, 'ObstacleAndCB_16.png', 'Resolution', 300);

    % For layer 32:
    plot_robot_and_obstacle(A_pts_rot(:,:,32), B_pts,dA);
    exportgraphics(gcf, 'RobotObstacle_32.png', 'Resolution', 300);

    plot_CB_single_theta(B_pts, Layers_vert(:,1:max_vert(32),32));
    exportgraphics(gcf, 'ObstacleAndCB_32.png', 'Resolution', 300);


    % Plot 3D surface:
    figure; hold on; grid on;
    title('C-Obstacle Surface over \theta');
    xlabel('d_x'); ylabel('d_y'); zlabel('\theta');
    colormap turbo;
    
    thetas = 0:2*pi/N:pi/2; % Stops at 90 deg

    N = length(thetas);

    for i = 1:N
        verts = Layers_vert(:, 1:max_vert(i), i);  % Only valid vertices
        k = max_vert(i);
        theta_i = thetas(i);

        fill3(verts(1,[1:k 1]), ...                 % x
              verts(2,[1:k 1]), ...                 % y
              theta_i * ones(1, k+1), ...           % z (constant)
              theta_i, ...                          % color
              'FaceAlpha', 0.5, 'EdgeColor', 'k');
    end

    view(3); axis tight; axis equal;
    set(gcf, 'Renderer', 'painters');
    set(gcf, 'Position', [100, 100, 1200, 900]);
    exportgraphics(gcf, 'CB_in_3D.png', 'Resolution', 300);
end