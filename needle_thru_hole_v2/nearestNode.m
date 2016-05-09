%% nearestNode.m
% This is the function to find the nearest node to q_rand node on the current tree
% Input: 1. the current tree
%        2. the randomly generated free collision node, q_rand
% 
% Output: 1. the nearest node on current tree
%         2. the shorest normalized distance

%% nearestNode.m function starts below
function [ q_near, q_near_index, normalized_dist_min] = nearestNode(tree_node_index, tree_node_config, q_rand)
    node_num = length(tree_node_index);
    normalized_dist_vec = [];
    for i = 1:node_num
        % calculate dist btw q_rand and current selected existed onde on tree
        normalized_dist = calNormalizedDist(q_rand, tree_node_config(:,i));
        % store this dist in vector
        normalized_dist_vec = [normalized_dist_vec; normalized_dist];
    end
    
    [normalized_dist_min, index] = min(normalized_dist_vec);
    q_near = tree_node_config(:,index);
    q_near_index = index;
end

%% subfunction to calculate the normalized distance btw q_rand and q_selected
function normalized_dist = calNormalizedDist(q_rand, q_selected)
    x_rand = q_rand(1);
    y_rand = q_rand(2);

    x_selected = q_selected(1);
    y_selected = q_selected(2);
    
    dist_square = ((x_rand - x_selected)^2 + (y_rand - y_selected)^2);
    normalized_dist = sqrt(dist_square);
end