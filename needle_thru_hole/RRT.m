function [ tree ] = RRT( initial_config, goal_config, max_nodes, max_iter, step_size, map_info)
    
    tree = struct('nodeIndex',[],'nodeConfig',[]);
    tree(1).nodeIndex = 1;
    tree(1).nodeConfig = initial_config;
    
    x_resolution = 0.2;
    y_resolution = 0.2;
    theta_resolution = 0.2;
    resolution = [x_resolution y_resolution theta_resolution];
    
    for i = 1:max_iter
        rnd_num = rand();
        if (rnd_num < 0.05)
            % 5% chance to sample goal_config
            q_rand = goal_config;
        else
            % 95% chance to get other collision free random sample
            q_rand = collisionFreeRandomConfig(rnd_num, map_info);
        end
        extendRRT(tree, q_rand, step_size, resolution, map_info);
     end
end

