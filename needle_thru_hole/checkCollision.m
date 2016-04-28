%% checkCollision.m
% This function is used to check if needle semicircle curve intersected
% with the straight line e.g., the wall or object
% Input: 1. the straight to check
%        2. the needle struc
%		 3. the transformation matrix from needle to tissue
% 
% Output: boolean value which represents collided or not.

%% Main funtion
function checkCollision( structStraightLine, structNeedleGeometry, trans_mat)
%CHECKCOLLISION Summary of this function goes here
%   Detailed explanation goes here
    
    radius = structNeedleGeometry.radius;
    point_start = structStraightLine.start;
    point_end = structStraightLine.end;

    point_center = trans_mat.trans;

    syms lambda;

    point_diff = lambda * point_end + (1 - lambda) * point_start - point_center;

    poly_eqn = (norm(point_diff))^2 - radius^2; % define the polynomial eqn about variable lambda
    
    poly_eqn = sym2poly(poly_eqn);
    % poly_eqn = collect(poly_eqn, lambda); % simplify it


end