function [a,b,c] = calPolyCoeff(point_start, point_end, point_center, radius)
    x_01 = point_start(1);
    x_02 = point_start(2);
    x_11 = point_end(1);
    x_12 = point_end(2);
    
    p_1 = point_center(1);
    p_2 = point_center(2);
    
    a = x_01^2 - 2*x_01*x_11 + x_02^2 - 2*x_02*x_12 + x_11^2 + x_12^2;
    b = 2*p_1*x_01 + 2*p_2*x_02 - 2*p_1*x_11 - 2*p_2*x_12 + 2*x_01*x_11 + 2*x_02*x_12 - 2*x_01^2 - 2*x_02^2;
    c = p_1^2 - 2*p_1*x_01 + p_2^2 - 2*p_2*x_02 - radius^2 + x_01^2 + x_02^2;
end

