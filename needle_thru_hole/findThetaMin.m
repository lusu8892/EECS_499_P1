%% findThetaMin.m
function theta_min = findThetaMin(theta_1, theta_2)
    a = (theta_1 - theta_2) ^ 2;
    b = (theta_1 - theta_2 - 2*pi) ^ 2;
    c = (theta_1 - theta_2 + 2*pi) ^ 2;
    theta_min = min([a b c]);
end
