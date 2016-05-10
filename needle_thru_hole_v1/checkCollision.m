%% checkCollision.m
% This function is used to check if needle semicircle curve intersected
% with the straight line e.g., the wall or object
% Input: 1. the obstacle
%        2. the node configuration
% 
% Output: boolean value which represents collided or not.
%         false: no intersection with semicircle curve
%         true: intersected with semicircle curve

%% checkCollision.m function starts below
function collision  = checkCollision(obstacle, node_config)
    % needle structure
    radius = 10;
    
    % obstacle info expressed as line by defining starting and ending point
    point_start = obstacle(:,1:3)';
    point_end = obstacle(:,4:6)';

    point_center(:,1) = node_config;
    theta = node_config(4);
    
    trans_mat_rot= [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];

    syms lambda real;

    point_diff = lambda * point_end + (1 - lambda) * point_start - point_center(1:3,1);

    poly_eqn =  expand(point_diff' * point_diff - radius^2); % define the polynomial eqn about variable lambda
    poly_eqn = collect(poly_eqn, lambda); % simplify it
    poly_coeff = sym2poly(poly_eqn); % return the coefficient of the poly eqn
    
    a = poly_coeff(1);
    b = poly_coeff(2);
    c = poly_coeff(3);
    
    eps = 10e-8;
    
    delta = b^2 - 4 * a * c; % discrminant of polynomial eqn
    point_on_line = [];
    if (delta > eps) % two intersectioin with circle
        poly_roots = solve(poly_eqn);
        if (poly_roots(1) >= 0 && poly_roots(1) <= 1) &&... % with two points on 
             (poly_roots(2) >= 0 && poly_roots(2) <= 1)
            lambda = poly_roots;
            point_on_line(:,1) = lambda(1) .* point_end + (1 - lambda(1)) .* point_start;
            point_on_line(:,2) = lambda(2) .* point_end + (1 - lambda(2)) .* point_start;
            
            check_result(1) = checkAngle(point_on_line(:,1), point_center(1:3,1), trans_mat_rot);
            check_result(2) = checkAngle(point_on_line(:,2), point_center(1:3,1), trans_mat_rot);

            if (check_result(1) >= 0 || check_result(2) >= 0)
                collision = true; % intersected
            else
                collision = false; % not intersected
            end    
        elseif (poly_roots(1) >= 0 && poly_roots(1) <= 1) ||...
             (poly_roots(2) >= 0 && poly_roots(2) <= 1)

            if (poly_roots(1) >= 0 && poly_roots(1) <= 1)
                lambda = poly_roots(1);
                point_on_line = lambda * point_end + (1 - lambda) * point_start;            
            elseif (poly_roots(2) >= 0 && poly_roots(2) <= 1)
                lambda = poly_roots(2);
                point_on_line = lambda * point_end + (1 - lambda) * point_start;
            end

            check_result = checkAngle(point_on_line, point_center(1:3,1), trans_mat_rot);
            if (check_result >= 0)
                collision = true; % intersected
            else
                collision = false; % not intersected
            end
            
        else
            collision = false;    
        end
    elseif (delta < -eps) % if delta < 0 no intersection
        collision = false;

    elseif (delta > -eps && delta < eps) % one intersection
        poly_roots = solve(poly_eqn);
        lambda = poly_roots;
        point_on_line = lambda * point_end + (1 - lambda) * point_start;
        
        check_result = checkAngle(point_on_line, point_center(1:3,1), trans_mat_rot);
        if (check_result >= 0)
            collision = true; % intersected
        else
            collision = false; % not intersected
        end
                  
    else % otherwise there is no intersecion
        collision = false;
    end

end

%% subfunction 
function [check_result] = checkAngle( point_1, point_2, trans_mat_rot)
    vec = point_1 - point_2;
    y_vec = trans_mat_rot * [0;1;0];
    
    check_result = dot(vec, y_vec);
end