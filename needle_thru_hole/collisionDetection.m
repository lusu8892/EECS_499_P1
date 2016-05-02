function [ check_result_list ] = collisionDetection( map_info, sample_node )
%COLLISIONDETECTION Summary of this function goes here
%   Detailed explanation goes here
    collision = false;
    check_result_list = [];
    for i = 1 : length(map_info)
        check_result_list(i,:) = checkCollision(map_info(i), sample_node);
        if (check_result_list(i,:) == true)
            break;
        else
            continue;
        end
    end
    
    if (isequal(check_result_list, zeros(length(map_info),1)))
%             sample_node.position = [x;y;z];
%             sample_node.direction = theta;
            return;
        else
            continue;
        end

end

