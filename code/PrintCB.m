% Print Obstacle & Mark are of CB of each obstacle:
for k = 1:nObst
    Obs_pts = Obstacles(:,:,k);
    CB_Obs_Pshape = CB_theta0_Poly_list{k,i};
    hold on; axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot Obstacle B
    patch(Obs_pts(1,:), Obs_pts(2,:), [1 0.6 0.6], ...
        'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);


    % Plot CB-Obstacle
    plot(CB_Obs_Pshape, 'FaceColor', 'none', ...
        'EdgeColor', color_matrix(4:6,k)', 'LineWidth', 1.5, 'LineStyle', '--');


    % Final touches
    grid on;
    xlabel('X'); ylabel('Y');
end

number_Union = size(CB_tot_perLayer,1);
for j = 1:number_Union
    poly_ij = CB_tot_perLayer{j,i};
    if isempty(poly_ij)
            continue;
    else
    axis equal;
    set(gcf, 'Renderer', 'painters');

    % Plot the polyshape
    plot(poly_ij, 'FaceColor', [1 0.6 0.6], 'EdgeColor', 'r', 'LineWidth', 1.5);

    % Apply zoom if specified
    xlim([-1,33]);
    ylim([-1,31]);

    % Final touches
    grid on;
    xlabel('X'); ylabel('Y');
    hold on;
    end
end