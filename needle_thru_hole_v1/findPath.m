%% findPath.m
% This is the function to find a possible path from start point to goal
% Input: 1. complete tree_node_index
%        2. complete tree_node_config
%        3. complete tree_parent_index
% 
% Output: 1. modified tree_node_index
%         2. modified tree_node_config
%         3. modified tree_parent_index
function [path_node_config, path_node_index] = findPath( tree_node_index, tree_node_config, tree_parent_node_index )
    path_node_index = []; % the array to store the node index on the path
    path_node_config = []; % the array to store the node config on the path
    parent_node_index = tree_parent_node_index(end); % this is the goal's parent node index
    
    path_node_index = [path_node_index; tree_node_index(end)];
    path_node_index = [path_node_index; parent_node_index];
    path_node_config(:,1) = [path_node_config tree_node_config(:,end)];
    
    while(~(isequal(path_node_config(:,end),tree_node_config(:,1))))
        node_config_on_path = tree_node_config(:,parent_node_index);
        path_node_config = [path_node_config node_config_on_path];
        path_node_index = [path_node_index; parent_node_index];
        parent_node_index = tree_parent_node_index(parent_node_index);
    end

end

