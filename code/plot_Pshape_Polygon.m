function plot_Pshape_Polygon(pgon)
    % pgon: a polyshape object (may include holes)
    
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot the polyshape
    plot(pgon, 'FaceColor', [1 0.6 0.6], 'EdgeColor', 'r', 'LineWidth', 1.5);

    % Final touches
    grid on;
    xlabel('X'); ylabel('Y');
    title('Polygon (polyshape) with possible holes');
end
