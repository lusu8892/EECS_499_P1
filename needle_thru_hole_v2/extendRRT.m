%% extendRRT.m
% This is the function to extend the tree

function [tree_node_index, tree_node_config, tree_parent_node_index] = ...
            extendRRT( tree_node_index, tree_node_config, tree_parent_node_index, q_rand, step_size, map_info, resolution)

    % find the closest neighbor of q among all nodes on current Tree
    [q_near, q_near_index, normalized_dist_min] = nearestNode(tree_node_index, tree_node_config, q_rand);
    
    % progress q_near by step_size along the straight line in Q btw q_near and q_rand
    [q_new, flag] = getNewNode(q_near, q_rand, step_size, normalized_dist_min);
    if (flag == 0) % this mean q_new is generated one step along q_near and q_rand
        collision = collisionDetection(map_info, q_new);
%         collision = false;
        if (collision == true)
            tree_node_index = tree_node_index;
            tree_node_config = tree_node_config;
            tree_parent_node_index = tree_parent_node_index;
            return; % tree keeping same as input
        else
            ons_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution);
%             ons_step_sz_collision = false;
            if (ons_step_sz_collision == true)
                tree_node_index = tree_node_index;
                tree_node_config = tree_node_config;
                tree_parent_node_index = tree_parent_node_index;
                return; % tree keeping same as input
            else
                [tree_node_index, tree_node_config, tree_parent_node_index] = ...
                    insertNodeToTree( tree_node_index, tree_node_config, tree_parent_node_index, q_near_index, q_new);
            end
        end
    else % this mean q_new = q_rand
        ons_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution);
%         ons_step_sz_collision = false;
        if (ons_step_sz_collision == true)
            tree_node_index = tree_node_index;
            tree_node_config = tree_node_config;
            tree_parent_node_index = tree_parent_node_index;
            return; % tree keeping same as input
        else
            [tree_node_index, tree_node_config, tree_parent_node_index] = ...
                   insertNodeToTree( tree_node_index, tree_node_config, tree_parent_node_index, q_near_index, q_new);
        end
    end
end

%% subfunction to check one step size thru
function one_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution)
    one_step_sz_collision = true;
    [num_of_steps, delta_x, delta_y] = getNumOfStepsAndIncrement(q_new, q_near, resolution);
    
    q_new_insertion = q_near;
    
    for i = 1:num_of_steps
        q_new_insertion(1,:) = q_new_insertion(1) + delta_x;
        q_new_insertion(2,:) = q_new_insertion(2) + delta_y;
        q_new_insertion(3,:) = 0;
        
        collision = collisionDetection( map_info, q_new_insertion);
        if (collision == true)
            return;
        end
    end
    
    one_step_sz_collision = false;
end

%% subfunction to find number of steps to check collision in one step size
function [num_of_steps, delta_x, delta_y] = getNumOfStepsAndIncrement(q_new, q_near, resolution)
    x_dist = q_new(1) - q_near(1);
    y_dist = q_new(2) - q_near(2);
%     theta_dist = q_new.direction - q_near.direction;
%     
%     if (abs(theta_dist) <= min(abs(theta_dist + 2*pi), abs(theta_dist - 2*pi)))
%         % keep thata_dist as previous
%     elseif (abs(theta_dist + 2*pi) <= min(abs(theta_dist), abs(theta_dist - 2*pi)))
%         theta_dist = theta_dist + 2*pi;
%     else
%         theta_dist = theta_dist - 2*pi;
%     end
    
    x_resolution = resolution(1);
    y_resolution = resolution(2);
%     theta_resolution = resolution(3);
    
    a = abs(x_dist) / x_resolution; 
    b = abs(y_dist) / y_resolution;
%     c = abs(theta_dist) / theta_resolution;
    num_of_steps = ceil(max([a b]));
    delta_x = x_dist / num_of_steps;
    delta_y = y_dist / num_of_steps;
%     delta_theta = theta_dist / num_of_steps; 
end
%% subfunction to get new node
function [q_new, flag] = getNewNode(q_nearest, q_rand, step_size, distance)
    if ( distance < step_size )
        flag = 1; % q_new = q_rand
        q_new = q_rand;
    else
        flag = 0; % q_new = new node
        
        vec = q_rand - q_nearest;
        vec = vec / norm(vec);
        
        q_new = q_nearest + vec * step_size;
    end
end

%% subfunction insert
function [tree_node_index, tree_node_config, tree_parent_node_index] = ...
            insertNodeToTree( tree_node_index, tree_node_config, tree_parent_node_index, q_parent_index, q_new)
        
    tree_node_index = [tree_node_index; length(tree_node_index) + 1];
    tree_node_config = [tree_node_config q_new];
    tree_parent_node_index = [tree_parent_node_index; q_parent_index];
    
end