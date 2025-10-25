function grid = make_simple_line(N, P1, P2)
    
    % Decide to do snake (1) or not (0):
    Snake = 1;

    % Initialize NxN grid of zeros
    grid = zeros(N);

    % Extract coordinates
    x1 = round(P1(1)); y1 = round(P1(2));
    x2 = round(P2(1)); y2 = round(P2(2));

    % Bresenham setup
    Deltax = x2 - x1;
    Deltay = y2 - y1;
    dx = abs(Deltax);
    dy = abs(Deltay);
    sx = sign(Deltax);
    sy = sign(Deltay);
    
    theta = mod(atan2d(Deltay, Deltax), 360);  % Angle in degrees [0, 360)

    x = x1;
    y = y1;

    % Driving axis
    if dx > dy
        err = dx / 2;
        for i = 1:(dx + 1)
            xi = round(x);
            yi = round(y);
            x = x + sx;
            err = err - dy;
            if err < 0
                y = y + sy;
                err = err + dx;
            end
            xi_next = round(x);
            yi_next = round(y);
            
            if xi >= 1 && xi <= N && yi >= 1 && yi <= N
                if Snake == 1
                    if xi~=x2 && yi~=y2  && (xi_next~=xi && yi_next~=yi)
                        if 0<theta && theta<90
                            grid(yi, xi+1) = 1;
                        elseif 90<theta && theta<180
                            grid(yi+1, xi) = 1;
                        elseif 180<theta && theta<270
                            grid(yi, xi-1) = 1;
                        elseif 270<theta && theta<360
                            grid(yi-1, xi) = 1;
                        end
                    end
                end
                grid(yi, xi) = 1;
            end 
        end
    else
        err = dy / 2;
        for i = 1:(dy + 1)
            xi = round(x);
            yi = round(y);
            y = y + sy;
            err = err - dx;
            if err < 0
                x = x + sx;
                err = err + dy;
            end
            xi_next = round(x);
            yi_next = round(y);
            if xi >= 1 && xi <= N && yi >= 1 && yi <= N
                if Snake == 1
                    if xi~=x2 && yi~=y2  && (xi_next~=xi && yi_next~=yi)
                        if 0<theta && theta<90
                            grid(yi, xi+1) = 1;
                        elseif 90<theta && theta<180
                            grid(yi+1, xi) = 1;
                        elseif 180<theta && theta<270
                            grid(yi, xi-1) = 1;
                        elseif 270<theta && theta<360
                            grid(yi-1, xi) = 1;
                        end
                    end
                end
                grid(yi, xi) = 1;
            end 
        end
    end
end