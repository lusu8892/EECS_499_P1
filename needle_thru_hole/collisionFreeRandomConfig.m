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
    sample_node.position = [];
    sample_node.direction = 0;

    while (true)
       
        
        % based on map info generate random sample configuration and check collision
        sample_node.position(1,:) = 100 * rand();
        sample_node.position(2,:) = 100 * rand();
        sample_node.position(3,:) = 0;
        sample_node.direction = (2 * pi) * rand();
        plot(sample_node.position(1,:),sample_node.position(2,:),'o');
        hold on
        pause(1);
        
%         collision = collisionDetection(map_info, sample_node);
%         
%         if (collision == true)
%             continue;
%         else
%             return;
%         end
    end
end