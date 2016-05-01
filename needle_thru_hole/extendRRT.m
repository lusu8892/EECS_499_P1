%% extendRRT.m
% This is the function to extend the tree

function q_new = extendRRT( tree, q_rand, step_size)
    
    
    [q_near, dist_min] = nearestNode(tree, q_rand);
    
    if (dist_min > step)
        q_new = getNewNode(q_near, step_size);
    else
    
    end

end

function q_new = getNewNode(q_near, step_size)
    
end