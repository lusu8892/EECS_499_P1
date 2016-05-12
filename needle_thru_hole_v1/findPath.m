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
    
    % put goal's index at the first node index on the path (need swap)
    path_node_index = [path_node_index; tree_node_index(end)];
    
%     % put goal's parent node index as second node index on the path
%     path_node_index = [path_node_index; parent_node_index];
    
    % put goal config as first node config on the path
    path_node_config(:,1) = [path_node_config tree_node_config(:,end)]; %
    
    while(~(isequal(path_node_config(:,end),tree_node_config(:,1))))
        node_config_on_path = tree_node_config(:,parent_node_index);
        path_node_config = [path_node_config node_config_on_path];
        path_node_index = [path_node_index; parent_node_index];
        parent_node_index = tree_parent_node_index(parent_node_index);
    end
    
    x_coordinate_path_node = path_node_config(1,:)';
    y_coordinate_path_node = path_node_config(2,:)';
    theta_angle_path_node = path_node_config(4,:)';
    plot3(x_coordinate_path_node, y_coordinate_path_node, theta_angle_path_node, 'color', 'b', 'lineWidth', 2);

end

