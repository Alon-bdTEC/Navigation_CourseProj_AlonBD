function plot_CB_single_theta(B_pts, C_obs)
    % Plots Obstacle B and C-Obstacle polygon at a given theta

    figure; hold on; grid on;
    title('Obstacle B and CB at Specific \theta');
    xlabel('x'); ylabel('y');

    % 1. Draw Obstacle B (gray polygon)
    fill(B_pts(1,[1:end 1]), B_pts(2,[1:end 1]), [0.6 0.6 0.6], ...
         'FaceAlpha', 0.5, 'EdgeColor','k');
    plot(B_pts(1,:), B_pts(2,:), 'ko', 'MarkerFaceColor', 'k');

    % 2. Draw C-obstacle
    C_obs_closed = [C_obs, C_obs(:,1)];
    plot(C_obs_closed(1,:), C_obs_closed(2,:), 'r-', 'LineWidth', 2);
    plot(C_obs(1,:), C_obs(2,:), 'ko', 'MarkerFaceColor', 'm');

    axis equal;
    set(gcf, 'Renderer', 'painters');
    % set(gcf, 'Position', [100, 100, 1200, 900]);
end
