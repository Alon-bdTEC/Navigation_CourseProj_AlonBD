function vertices = calc_CB_th_0(A_pts, B_pts)
    % calc_CB: finds CB for given vertices
    %
    % Inputs:
    %   A_pts = [a1, a2,..] - 2xnA
    %
    %   B_pts = [b1, b2,..] - 2xnB
    %
    %   CCW counts of robot A and obstalce B vertices seperatly
    %
    % Output: Returns vertices of CB of A and B, for given vertices numbers
    %

    % -----------Defining Parameters-----------
    % Define nA and nB:
    nA = size(A_pts,2);
    nB = size(B_pts,2);

    % -----------Get normals of A and B-----------
    % Get normals of A
    nA_s = zeros(2,nA);

    for i = 1:nA
        if i ~= nA
            p1 = A_pts(:,i);
            p2 = A_pts(:,i+1);
        else
            p1 = A_pts(:,nA);
            p2 = A_pts(:,1);
        end
        nA_s(:,i) = -normal_bet_points(p1,p2);
    end
    
    % Normals of B
    nB_s = zeros(2,nB);

    for i = 1:nB
        if i ~= nB
            p1 = B_pts(:,i);
            p2 = B_pts(:,i+1);
        else
            p1 = B_pts(:,nB);
            p2 = B_pts(:,1);
        end
        nB_s(:,i) = normal_bet_points(p1,p2);
    end

    % -----------Extract angles Between Normal Vectors-----------

    angles_nA = zeros(2,length(nA_s));
    
    for i = 1:nA
        if i == 1
            angles_nA(:,i) = [vector_angle(nA_s(:,nA-(i-1)));vector_angle(nA_s(:,i))]; 
        else
            angles_nA(:,i) = [vector_angle(nA_s(:,(i-1)));vector_angle(nA_s(:,i))]; 
        end
    end
    
    angles_nB = zeros(2,length(nB_s));
    
    for i = 1:nB
        if i == 1
            angles_nB(:,i) = [vector_angle(nB_s(:,nB-(i-1)));vector_angle(nB_s(:,i))]; 
        else
            angles_nB(:,i) = [vector_angle(nB_s(:,(i-1)));vector_angle(nB_s(:,i))]; 
        end
    end

    % -----------Find Overlaps Bet. A and B, in CCW order, and store i,j indeces-----------
    d_ij = zeros(2,nA+nB);
    w=1;
    
    for j = 1:nB
        
        % Count vertices-vertices touches per b_j
        a_i_rel_b_j = [];
        indices= [];
        count = 0;
        index_start = 1;
    
        max_ang = 0;
        for k = 1:nA
            if segments_int(angles_nB(:,j), angles_nA(:,k))
                count = count + 1;
                a_i_rel_b_j(:,count) = angles_nA(:,k) - angles_nB(1,j);
                
                % Normalize all angles to [0, 2*pi)
                a_i_rel_b_j = mod(a_i_rel_b_j + 2*pi, 2*pi);
    
                indices(count) = k;
                if count == 1
                    max_ang = a_i_rel_b_j(1,count);
                    index_start = k;
                elseif (a_i_rel_b_j(1,count) >= max_ang)
                    max_ang = a_i_rel_b_j(1,count);
                    index_start = k;
                end
            end
        end
    
        n_A_indeces = mod((index_start-1):(index_start+count-2), nA) + 1;

        for k = 1:count
            d_ij(:,w) = [n_A_indeces(k);j];
            w = w + 1;
        end 
    
    end
    
    % disp('Intersecting [b_j; a_i]:');
    % disp(d_ij);

    % -----------Getting rid of 3/more collinear points in CB-----------
    % CB points
    C_obs = zeros(2,nA+nB);
    
    for k = 1:size(d_ij, 2)
        i = d_ij(1,k);  % a_i index
        j = d_ij(2,k);  % b_j index
    
        % Compute dij = b_j - a_i
        dij = B_pts(:,j) - A_pts(:,i);
        
        % Round very-integery numbers:
        if abs(dij(1) - round(dij(1))) < 1e-8
            dij(1) = round(dij(1));
        end
        if abs(dij(2) - round(dij(2))) < 1e-8
            dij(2) = round(dij(2));
        end

        C_obs(:,k) = dij;
    end

    % Getting rid of unnecessary points
    n = size(C_obs, 2);
    keep = true(1, n);

    for i = 1:n
        p1 = C_obs(:, mod(i-2, n) + 1);
        p2 = C_obs(:, i);
        p3 = C_obs(:, mod(i, n) + 1);

        v1 = p2 - p1;
        v2 = p3 - p2;
        if abs(det([v1 v2])) < 1e-8  % 2D linear-dependence check
            keep(i) = false;
        end
    end

    C_obs = C_obs(:, keep);

    vertices = C_obs;
    
end