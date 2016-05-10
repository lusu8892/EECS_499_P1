%% collisionFreeRandomConfig.m
% This function is used to check if needle semicircle curve intersected
% with the straight line e.g., the wall or object
% Input: 1. the straight to check
%        2. the needle struc
%		 3. the transformation matrix from needle to tissue
% 
% Output: boolean value which represents collided or not.
%         false: no intersection with semicircle curve
%         true: intersected with semicircle curve

%% main function
function [ sample_node ] = collisionFreeRandomConfig(map_info)
    
    while (true)    
        % based on map info generate random sample configuration and check collision
        sample_node(1,:) = 100 * rand();
        sample_node(2,:) = 100 * rand();
        sample_node(3,:) = 0;
        sample_node(4,:) = (2 * pi) * rand();
        
        collision = collisionDetection(map_info, sample_node);
        
        if (collision == true)
            continue;
        else
            return;
        end
    end
end