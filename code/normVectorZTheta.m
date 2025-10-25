function norm_wtheta = normVectorZTheta(Vi,Vj)
    X_j = Vj(1);
    X_i = Vi(1);
    Y_j = Vj(2);
    Y_i = Vi(2);
    Z_j = Vj(3);
    Z_i = Vi(3);
    
    % Option 1:
    angled_dif = abs(Z_j - Z_i);
    norm_wtheta = abs(X_j-X_i)+abs(Y_j-Y_i) +...
            abs(min(angled_dif,2*pi-angled_dif));
    
    % Option 2:
    % norm_wtheta = sqrt( (X_j-X_i)^2 + (Y_j-Y_i)^2 + (Z_j-Z_i)^2 );

end