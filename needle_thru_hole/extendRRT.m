%% extendRRT.m
% This is the function to extend the tree

function q_new = extendRRT( tree, q_rand, step_size, map_info)
    
    % find the closest neighbor of q among all nodes on current Tree
    [q_near, dist_min] = nearestNode(tree, q_rand);
    
    if (dist_min > step_size)
        % progress q_near by step_size along the straight line in Q btw
        % q_near and q_rand
        q_new = getNewNode(q_near, q_rand, step_size);
        checkCollision
    else
        q_new = q_rand;
    end

end

function q_new = getNewNode(q_near, q_rand, step_size)
    increase_dir = q_rand.position - q_near.position;
    increase_dir = increase_dir / norm(increase_dir);
    
    theta_rand = q_rand.direction;
    theta_near = q_near.direction;
    
    delta_theta_min = findThetaMin(theta_rand, theta_near);
    delta_theta_min = delta_theta_min/norm(delta_theta_min);
    
    
end