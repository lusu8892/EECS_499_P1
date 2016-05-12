syms lambda x_11 x_12 x_01 x_02 p_1 p_2 radius real;

point_end = [x_11 x_12 0]';
point_start = [x_01 x_02 0]';

point_center = [p_1 p_2 0]';

% radius = 10;

point_diff = lambda * point_end + (1 - lambda) * point_start - point_center(1:3,1);

poly_eqn =  expand(point_diff' * point_diff - radius^2); % define the polynomial eqn about variable lambda
poly_eqn = collect(poly_eqn, lambda); % simplify it
poly_coeff = sym2poly(poly_eqn); % return the coefficient of the poly eqn