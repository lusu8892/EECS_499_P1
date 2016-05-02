%% collisionDetection.m
% This function is used to check if needle semicircle curve intersected
% with all the obstacle in given map
% Input: 1. map information
%        2. the node configuration
% 
% Output: boolean value which represents collided or not.
%         false: no intersection with semicircle curve
%         true: intersected with semicircle curve
%% collisionDetection.m starts below
function [ collision ] = collisionDetection( map_info, sample_node )
    collision = true;
%     collision_check_list = [];
    for i = 1 : length(map_info)
        collision_check = checkCollision(map_info(i), sample_node);
        if (collision_check == true)
            return;
        else
            continue;
        end
    end
    
    collision = false;
    
%     if (isequal(collision_check_list, zeros(length(map_info),1)))
% %             sample_node.position = [x;y;z];
% %             sample_node.direction = theta;
%             return;
%         else
%             continue;
%         end

end

