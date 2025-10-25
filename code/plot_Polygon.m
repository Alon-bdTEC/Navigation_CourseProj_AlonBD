function plot_Polygon(P_pts)
    
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');
    
    % Plot Obstacle B
    fill(P_pts(1,:), P_pts(2,:), [1 0.6 0.6], ...
        'EdgeColor', 'b', 'LineWidth', 1.5);

    % Final touches
    grid on;
    xlabel('X'); ylabel('Y');
    title('Polygon in 2D');
    
end
