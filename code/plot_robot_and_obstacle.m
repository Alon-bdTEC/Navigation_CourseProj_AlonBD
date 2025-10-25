function plot_robot_and_obstacle(A_pts, B_pts,dA)
    axis_len = 2;
    d = dA;       % Translation vector
    m = 1;            % Margin value

    figure; hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot World Frame Fw
    origin = [0; 0];
    x_axis = [axis_len; 0];
    y_axis = [0; axis_len];

    quiver(origin(1), origin(2), x_axis(1), x_axis(2), 0, ...
        'Color', [1 0.5 0], 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
    quiver(origin(1), origin(2), y_axis(1), y_axis(2), 0, ...
        'Color', [1 0.5 0], 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
    text(origin(1) + 0.1, origin(2) + 1, 'F_w', ...
        'Color', [1 0.5 0], 'FontWeight', 'bold');

    % Translate Robot A by d
    A_pts_trans = A_pts + d;
    % A_pts_trans = A_pts + (B_pts(:,1)-A_pts(:,3)); % FAKE Translation

    % Plot Robot A
    fill(A_pts_trans(1,:), A_pts_trans(2,:), [0.6 0.8 1], ...
        'EdgeColor', 'k', 'LineWidth', 1.5);

    nA = size(A_pts_trans, 2);
    label_color = [0 0 0.5]; % Dark blue

    for i = 1:nA
        text(A_pts_trans(1,i) + 0.15, A_pts_trans(2,i) + 0.15, ...
            sprintf('a_{%d}', i), 'Color', label_color, 'FontSize', 10);
    end

    % Add Robot Local Frame F_A at a_1
    a1 = A_pts_trans(:,1);
    a2 = A_pts_trans(:,2);
    dir_vec = a2 - a1;
    dir_vec = dir_vec / norm(dir_vec);
    x_FA = dir_vec * axis_len;
    y_FA = [-dir_vec(2); dir_vec(1)] * axis_len;

    quiver(a1(1), a1(2), x_FA(1), x_FA(2), 0, ...
        'Color', label_color, 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
    quiver(a1(1), a1(2), y_FA(1), y_FA(2), 0, ...
        'Color', label_color, 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
    text(a1(1) + 0.1, a1(2) - 0.4, 'F_A', ...
        'Color', label_color, 'FontWeight', 'bold');

    % Plot Obstacle B
    fill(B_pts(1,:), B_pts(2,:), [1 0.6 0.6], ...
        'EdgeColor', 'r', 'LineWidth', 1.5);

    nB = size(B_pts, 2);
    for j = 1:nB
        text(B_pts(1,j) + 0.15, B_pts(2,j) + 0.15, ...
            sprintf('b_{%d}', j), 'Color', 'r', 'FontSize', 10);
    end

    % -------- Add margin to plot --------
    all_pts = [A_pts_trans, B_pts, origin, origin + x_axis, origin + y_axis];
    min_vals = min(all_pts, [], 2) - m;
    max_vals = max(all_pts, [], 2) + m;
    xlim([min_vals(1), max_vals(1)]);
    ylim([min_vals(2), max_vals(2)]);

    % Final touches
    grid on;
    xlabel('X'); ylabel('Y');
    title('Robot A and Obstacle B in 2D');
    
end
