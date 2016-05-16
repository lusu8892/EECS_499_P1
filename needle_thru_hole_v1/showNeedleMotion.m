%% showNeedleMotion.m
% This function is used to show the needle motion given a path planning
% with the straight line e.g., the wall or object
% Input: 1. on path node configuration (needle configuration)
%        2. on path node index

%% showNeedleMotion.m function starts below
function showNeedleMotion( path_node_config, path_node_index, needle_radius )
    
    
    for i = 1:length(path_node_index)
        x = path_node_config(1,i);
        y = path_node_config(2,i);
        theta_1 = path_node_config(4,i);
        theta_2 = theta_1 + pi;
        
        circleDrawing(x,y,needle_radius,theta_1,theta_2);
    end
end

