function [ tree ] = RRT( initial_config, goal_config, max_nodes, max_iter, step_size )
    
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
            q_rand = goal_config;
        else
            q_rand = genRandomConfig(rnd_num);
        end
        extendRRT(tree, q_rand, step_size, resolution);
     end
end

