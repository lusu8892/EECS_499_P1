%% extendRRT.m
% This is the function to extend the tree
% Input: 1. pre-modified tree_node_index
%        2. pre-modified tree_node_config
%        3. pre-modified tree_parent_index
%        4. random sample node in Qfree
%        5. pre-defined step_size
%        6. resolution array
% 
% Output: 1. modified tree_node_index
%         2. modified tree_node_config
%         3. modified tree_parent_index


function [tree_node_index, tree_node_config, tree_parent_node_index] = ...
            extendRRT( tree_node_index, tree_node_config, tree_parent_node_index, q_rand, step_size, map_info, resolution)

    % find the closest neighbor of q among all nodes on current Tree
    [q_near, q_near_index, normalized_dist_min] = nearestNode(tree_node_index, tree_node_config, q_rand);
    
    % progress q_near by step_size along the straight line in Q btw q_near and q_rand
    [q_new, flag] = getNewNode(q_near, q_rand, step_size, normalized_dist_min);
    if (flag == 0) % this mean q_new is generated one step along q_near and q_rand
        collision = collisionDetection(map_info, q_new);
        if (collision == true)
            tree_node_index = tree_node_index;
            tree_node_config = tree_node_config;
            tree_parent_node_index = tree_parent_node_index;
            return; % tree keeping same as input
        else
            ons_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution);
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
    [num_of_steps, delta_x, delta_y, delta_theta] = getNumOfStepsAndIncrement(q_new, q_near, resolution);
    
    q_new_insertion = q_near;

    for i = 1:num_of_steps
        q_new_insertion(1,:) = q_new_insertion(1) + delta_x;
        q_new_insertion(2,:) = q_new_insertion(2) + delta_y;
        q_new_insertion(3,:) = 0;
        q_new_insertion(4,:) = q_new_insertion(4) + delta_theta;
        
        % correction angle to a right value
        if (q_new_insertion(4) > 2*pi)
            q_new_insertion(4,:) = q_new_insertion(4,:) - 2*pi;
        elseif (q_new_insertion(4) < 0)
            q_new_insertion(4,:) = q_new_insertion(4,:) + 2*pi;
        end
        
        collision = collisionDetection( map_info, q_new_insertion);
        if (collision == true)
            return;
        end
    end
    
    one_step_sz_collision = false;
end

%% subfunction to find number of steps to check collision in one step size
function [num_of_steps, delta_x, delta_y, delta_theta] = getNumOfStepsAndIncrement(q_new, q_near, resolution)
    x_dist = q_new(1) - q_near(1);
    y_dist = q_new(2) - q_near(2);
    theta_dist = q_new(4) - q_near(4);
    
    if (abs(theta_dist) <= min(abs(theta_dist + 2*pi), abs(theta_dist - 2*pi)))
        % keep thata_dist as previous
    elseif (abs(theta_dist + 2*pi) <= min(abs(theta_dist), abs(theta_dist - 2*pi)))
        theta_dist = theta_dist + 2*pi;
    else
        theta_dist = theta_dist - 2*pi;
    end
    
    x_resolution = resolution(1);
    y_resolution = resolution(2);
    theta_resolution = resolution(3);
    
    a = abs(x_dist) / x_resolution; 
    b = abs(y_dist) / y_resolution;
    c = abs(theta_dist) / theta_resolution;
    num_of_steps = ceil(max([a b c]));
    delta_x = x_dist / num_of_steps;
    delta_y = y_dist / num_of_steps;
    delta_theta = theta_dist / num_of_steps; 
end
%% subfunction to get new node
function [q_new, flag] = getNewNode(q_nearest, q_rand, step_size, distance)
    if ( distance < step_size )
        flag = 1; % q_new = q_rand
        q_new = q_rand;
    else
        q_new(1,:) = q_nearest(1) + (q_rand(1) - q_nearest(1)) / distance * step_size;
        q_new(2,:) = q_nearest(2) + (q_rand(2) - q_nearest(2)) / distance * step_size;
        q_new(3,:) = 0;
        if ( abs(q_rand(4) - q_nearest(4)) <= pi )
            q_new(4,:) = q_nearest(4) + (q_rand(4) - q_nearest(4)) / distance * step_size;
        else %if the angle difference is larger than 180 degrees, we will need to go the other way
             if (q_nearest(4) < q_rand(4))
                 theta_dist = q_rand(4) - q_nearest(4) - 2*pi;
                 delta_theta = theta_dist / distance * step_size;
                 new_theta = q_nearest(4) + delta_theta;
                 if (new_theta < 0) 
                     new_theta = new_theta + 2*pi;  %if angle became negative, we have to correct it.
                 end
             else
                 theta_dist = q_rand(4) - q_nearest(4) + 2*pi;
                 delta_theta = theta_dist / distance * step_size;
                 new_theta = q_nearest(4) + delta_theta;
                 if (new_theta > 2 * pi) 
                    new_theta = new_theta - 2*pi;  %if angle became larger than 2*pi, we have to correct it.
                 end
             end
             q_new(4,:) = new_theta;
        end
        flag = 0; % q_new = new node
    end
end

%% subfunction insert
function [tree_node_index, tree_node_config, tree_parent_node_index] = ...
            insertNodeToTree( tree_node_index, tree_node_config, tree_parent_node_index, q_parent_index, q_new)
    tree_node_index = [tree_node_index; length(tree_node_index) + 1];
    tree_node_config = [tree_node_config q_new];
    tree_parent_node_index = [tree_parent_node_index; q_parent_index];
end