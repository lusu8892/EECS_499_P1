%% NeedleThruHoleMainFunc.m
% This is the main function of the implemention of RRT Algorithm which is
% used to do the path planning for a needle thru a hole

function [ output_args ] = NeedleThruHoleMainFunc()
    %% Needle Type 3. non ideal curved Needle
    structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);

    %% set initial and goal transformation matrix
    % initial configuration
    needle_tip_pos_init.position = [30;20;0]; % the initial need tip position wrt spatial frame
    needle_tip_pos_init.direction = 0;

    initialTransMatFrmNeedleToTissue.rot = [cos(needle_tip_pos_init.direction) -sin(needle_tip_pos_init.direction) 0;...
                                            sin(needle_tip_pos_init.direction) cos(needle_tip_pos_init.direction) 0;0 0 1]; % the initial need frame orientation
    initialTransMatFrmNeedleToTissue.trans = needle_tip_pos_init.position - initialTransMatFrmNeedleToTissue.rot...
                                    * structNeedleGeometry.radius * [1 0 0]';
    % goal configuration
    needle_tip_pose_goal.position = [80;80;0];
    needle_tip_pose_goal.direction = 0;
    goalTransMatFrmNeedleToTissue.rot = [cos(needle_tip_pose_goal.direction) -sin(needle_tip_pose_goal.direction) 0;...
                                            sin(needle_tip_pose_goal.direction) cos(needle_tip_pose_goal.direction) 0;0 0 1];
    goalTransMatFrmNeedleToTissue.trans = needle_tip_pose_goal.position - goalTransMatFrmNeedleToTissue.rot...
                                    * structNeedleGeometry.radius * [1 0 0]';
    %% the number of attempts to expand the tree
%     MAX_NODE = 5000;
    MAX_ITER = 50;
    STEP_SIZE = 0.01;
    %% read in map info
    run('map_info_script');
    %% execute RRT
    [ tree ] = RRT( needle_tip_pos_init, needle_tip_pose_goal, MAX_ITER, STEP_SIZE, map_info);
    
end

