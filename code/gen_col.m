function color_matrix = gen_col(n)
    % Generate n distinct colors in HSV space
    hsv_colors = hsv(n);  % n x 3 matrix of base HSV colors
    color_matrix = zeros(6, n);

    for i = 1:n
        base_rgb = hsv_colors(i, :);  % Base color in RGB

        % Lighter version: blend with white
        lighter_rgb = min(base_rgb + 0.3, 1);

        % Darker version: blend with black
        darker_rgb = max(base_rgb - 0.3, 0);

        % Store in matrix
        color_matrix(1:3, i) = lighter_rgb';
        color_matrix(4:6, i) = darker_rgb';
    end
end