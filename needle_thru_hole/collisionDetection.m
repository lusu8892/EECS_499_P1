function [ collision ] = collisionDetection( map_info, sample_node )
%COLLISIONDETECTION Summary of this function goes here
%   Detailed explanation goes here
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

