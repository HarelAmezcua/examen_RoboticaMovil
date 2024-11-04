% computeVelocities.m
function [vp, vr,s] = computeVelocities(uv, Rt_pose, K, apriltagCornerPoints, cRp, ctp, lamda, sd)
    % To get Z
    Z = Rt_pose.A * apriltagCornerPoints;
    Z = Z(3, :);

    % 2D projection
    s1 = Proyection_2D(uv(1, :), K);
    s2 = Proyection_2D(uv(2, :), K);
    s3 = Proyection_2D(uv(3, :), K);
    s4 = Proyection_2D(uv(4, :), K);
    s = [s1 s2 s3 s4]';    

    % Interaction matrix
    L1 = Interaction_Matrix(s1, Z(1));
    L2 = Interaction_Matrix(s2, Z(2));
    L3 = Interaction_Matrix(s3, Z(3));
    L4 = Interaction_Matrix(s4, Z(4));
    Ls = [L1; L2; L3; L4];

    V = [cRp ctp * cRp; zeros(3, 3) cRp];

    vp = -lamda * pinv(Ls * V) * (s - sd);
    
    v_local = vp(1);
    omega_local = vp(6);

    
    R = 0.195/2;
    L = 0.381;    

    wr = (2*v_local + omega_local*L)/2*R;
    wl = (2*v_local - omega_local*L)/2*R;

    vr = [wr;wl];
end
