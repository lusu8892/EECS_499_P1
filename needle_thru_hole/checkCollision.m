%% checkCollision.m
% This function is used to check if needle semicircle curve intersected
% with the straight line e.g., the wall or object
% Input: 1. the straight to check
%        2. the needle struc
%		 3. the transformation matrix from needle to tissue
% 
% Output: boolean value which represents collided or not.

%% Main funtion
function [ boolean ] = checkCollision( structStraightLine, structNeedleGeometry, trans_mat)
%CHECKCOLLISION Summary of this function goes here
%   Detailed explanation goes here
    
    radius = structNeedleGeometry.radius;
    point_start = structStraightLine.start;
    point_end = structStraightLine.end;

    point_center = trans_mat.trans;

    syms lambda real;

    point_diff = lambda * point_end + (1 - lambda) * point_start - point_center;

    poly_eqn =  expand(point_diff' * point_diff - radius^2); % define the polynomial eqn about variable lambda
    poly_eqn = collect(poly_eqn, lambda); % simplify it
    poly_coeff = sym2poly(poly_eqn); % return the coefficient of the poly eqn
    
    a = poly_coeff(1);
    b = poly_coeff(2);
    c = poly_coeff(3);
    
    delta = b^2 - 4 * a * c; % discrminant of polynomial eqn
    
    if (delta > 0)
        poly_roots = solve(poly_eqn);
        if ((poly_roots(1) >= 0 && poly_roots(1) <= 1) ||...
             (poly_roots(2) >= 0 && poly_roots(2) <= 1))
        else
            
         
    else

end