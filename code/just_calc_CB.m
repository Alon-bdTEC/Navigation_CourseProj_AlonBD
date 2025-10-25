function [max_vert,Layers_vert] = just_calc_CB(A_pts, B_pts,N)
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
    %   thetas in range 0->2pi-2pi/N in N 2pi/N steps
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
    
    % Define target angles as multiples of pi
    targets = [1/2, 1, 3/2];  % These represent pi/2, pi, 3pi/2

    % Initialize output
    angle_indices = -1 * ones(1, 3);

    for i = 1:3
        x = targets(i);  % e.g., 1/2 means pi/2
    
        % We want to solve: x*pi = k*step => k = (x*pi) / step = (x*pi) / (2*pi/N) = x * N / 2
        k = x * N / 2;
    
        if mod(k,1) == 0 && k >= 0 && k < N
            angle_indices(i) = k + 1;  % MATLAB indexing (1-based)
        end
    end

    A_pts_rot = zeros(2,nA);
    for i = 1:N

        theta = thetas(i);
        if i == 1
            R = eye(2);
        elseif angle_indices(1) ~= -1 && i==angle_indices(1)
            R = [0 -1;1,0];
        elseif angle_indices(2) ~= -1 && i==angle_indices(2)
            R = [-1 0;0 -1];
        elseif angle_indices(3) ~= -1 && i==angle_indices(3)
            R = [0 1;-1 0];
        else
            R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
        end
        
        A_pts_rot(:,:) = A_pts;

        for k = 1:nA
            A_pts_rot(:,k) = R*A_pts_rot(:,k);
        end
        
        Layer_i = calc_CB_th_0(A_pts_rot(:,:), B_pts);
        max_vert(i) = size(Layer_i,2);
        Layers_vert(:,1:max_vert(i),i) = Layer_i;
    end
    
end