% Proyection_2D.m
function s = Proyection_2D(uv, K)
    uv0 = K.PrincipalPoint;
    f = K.FocalLength;
    s = (uv - uv0) ./ f;
end
