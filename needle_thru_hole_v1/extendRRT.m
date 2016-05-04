%% extendRRT.m
% This is the function to extend the tree

function tree = extendRRT( tree, q_rand, step_size, map_info, resolution)

    % find the closest neighbor of q among all nodes on current Tree
    [q_near, q_near_index, normalized_dist_min] = nearestNode(tree, q_rand);
    
    % progress q_near by step_size along the straight line in Q btw q_near and q_rand
    [q_new, flag] = getNewNode(q_near, q_rand, step_size, normalized_dist_min);
    if (flag == 0) % this mean q_new is generated one step along q_near and q_rand
        collision = collisionDetection(map_info, q_new);
        if (collision == true)
            tree = tree;
            return; % tree keeping same as input
        else
            ons_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution);
            if (ons_step_sz_collision == true)
                tree = tree;
                return; % tree keeping same as input
            else
                tree = insertNodeToTree( tree, q_near, q_near_index, q_new);
            end
        end
    else % this mean q_new = q_rand
        ons_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution);
        if (ons_step_sz_collision == true)
            tree = tree;
            return; % tree keeping same as input
        else
            tree = insertNodeToTree( tree, q_near, q_near_index, q_new);
        end
    end
%     
%     if (isnan(q_new)) % if q_new == NaN then jump to another iteration
%         tree = tree;
%         return;
%     else
%         tree = insertNodeToTree( tree, q_near, q_new);
%     end
end

%% subfunction to check one step size thru
function one_step_sz_collision = checkOneStepSzThru(map_info, q_new, q_near, resolution)
    one_step_sz_collision = true;
    [num_of_steps, delta_x, delta_y, delta_theta] = getNumOfStepsAndIncrement(q_new, q_near, resolution);
    
    q_new_insertion.position(1,:) = q_near.position(1);
    q_new_insertion.position(2,:) = q_near.position(2);
    q_new_insertion.position(3,:) = 0;
    q_new_insertion.direction = q_near.direction;
        
    for i = 1:num_of_steps
        q_new_insertion.position(1,:) = q_new_insertion.position(1) + delta_x;
        q_new_insertion.position(2,:) = q_new_insertion.position(2) + delta_y;
        q_new_insertion.position(3,:) = 0;
        q_new_insertion.direction = q_new_insertion.direction + delta_theta;
        
        % correction angle to a right value
        if (q_new_insertion.direction > 2*pi)
            q_new_insertion.direction = q_new_insertion.direction - 2*pi;
        elseif (q_new_insertion.direction < 0)
            q_new_insertion.direction = q_new_insertion.direction + 2*pi;
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
    x_dist = q_new.position(1) - q_near.position(1);
    y_dist = q_new.position(2) - q_near.position(2);
    theta_dist = q_new.direction - q_near.direction;
    
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
        q_new.position(1,:) = q_nearest.position(1) + (q_rand.position(1) - q_nearest.position(1)) / distance * step_size;
        q_new.position(2,:) = q_nearest.position(2) + (q_rand.position(2) - q_nearest.position(2)) / distance * step_size;
        q_new.position(3,:) = 0;
        if ( abs(q_rand.direction - q_nearest.direction) <= pi )
            q_new.direction = q_nearest.direction + (q_rand.direction - q_nearest.direction) / distance * step_size;
        else %if the angle difference is larger than 180 degrees, we will need to go the other way
             if (q_nearest.direction < q_rand.direction)
                 theta_dist = q_rand.direction - q_nearest.direction - 2*pi;
                 delta_theta = theta_dist / distance * step_size;
                 new_theta = q_nearest.direction + delta_theta;
                 if (new_theta < 0) 
                     new_theta = new_theta + 2*pi;  %if angle became negative, we have to correct it.
                 end
             else
                 theta_dist = q_rand.direction - q_nearest.direction + 2*pi;
                 delta_theta = theta_dist / distance * step_size;
                 new_theta = q_nearest.direction + delta_theta;
                 if (new_theta > 2 * pi) 
                    new_theta = new_theta - 2*pi;  %if angle became larger than 2*pi, we have to correct it.
                 end
             end
             q_new.direction=new_theta;
        end
        flag = 0; % q_new = new node
    end
end

%% subfunction insert
function tree = insertNodeToTree( tree, q_parent, q_parent_index, q_new)
    tree.nodeIndex = [tree.nodeIndex; length(tree.nodeIndex) + 1];
    tree.nodeConfig = [tree.nodeConfig; q_new];
    tree.parentNodeIndex = [tree.parentNodeIndex; q_parent_index];
end