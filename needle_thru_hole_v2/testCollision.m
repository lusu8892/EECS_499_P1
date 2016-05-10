%% Needle Type 3. non ideal curved Needle
structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);

%% set initial and goal transformation matrix
% initial configuration
needle_tip_pos_init.position = [30;20;0]; % the initial need tip position wrt spatial frame
%     needle_tip_pos_init.direction = 0;

initialTransMatFrmNeedleToTissue.rot = [1 0 0;0 1 0;0 0 1]; % the initial need frame orientation
initialTransMatFrmNeedleToTissue.trans = needle_tip_pos_init.position - initialTransMatFrmNeedleToTissue.rot...
                                * structNeedleGeometry.radius * [1 0 0]';
initial = initialTransMatFrmNeedleToTissue.trans ;
% goal configuration
needle_tip_pose_goal.position = [80;80;0];
%     needle_tip_pose_goal.direction = 0;
goalTransMatFrmNeedleToTissue.rot = [1 0 0;0 1 0;0 0 1];
goalTransMatFrmNeedleToTissue.trans = needle_tip_pose_goal.position - goalTransMatFrmNeedleToTissue.rot...
                                * structNeedleGeometry.radius * [1 0 0]';
goal = goalTransMatFrmNeedleToTissue.trans;

%%
clf;
axis([0 100 0 100]);
axis square;
% Parameters
tissue = [0  0  100 100 0; 0 100 100 0 0];
wall_top = [45 45 55 55;100 55 55 100];
wall_bottom = [45 45 55 55;0 45 45 0];
% Plot the box.
line(tissue(1,:),tissue(2,:),'color','blue');
line(wall_top(1,:),wall_top(2,:),'color','blue');
line(wall_bottom(1,:),wall_bottom(2,:),'color','blue');
hold on;

%% map info
mapUpBound = horzcat([0; 100; 0]', [100; 100; 0]');
mapLeftBound = horzcat([0; 0; 0]', [0; 100; 0]');
mapBottomBound = horzcat([0; 0; 0]', [100; 0; 0]');
mapRightBound = horzcat([100; 0; 0]', [100; 100; 0]');

% define the boundry of the objects
% top wall
topWallRight = horzcat([45; 55; 0]', [45; 100; 0]');
topWallLeft = horzcat([55; 55; 0]', [55; 100; 0]');
topWallBottom = horzcat([45; 55; 0]', [55; 55; 0]');
% bottom wall
downWallRight = horzcat([45; 0; 0]', [45; 45; 0]');
downWallLeft = horzcat([55; 0; 0]', [55; 45; 0]');
downWallBottom = horzcat([45; 45; 0]', [55; 45; 0]');

% push back to map_info array
map_info = [];
map_info = [map_info; mapUpBound];
map_info = [map_info; mapLeftBound];
map_info = [map_info; mapBottomBound];
map_info = [map_info; mapRightBound];
map_info = [map_info; topWallRight];
map_info = [map_info; topWallLeft];
map_info = [map_info; topWallBottom];
map_info = [map_info; downWallRight];
map_info = [map_info; downWallLeft];
map_info = [map_info; downWallBottom];
%% testing starts
sample_node = [50;42;0];
h = circleDrawing(sample_node(1),sample_node(2),structNeedleGeometry.radius,-pi/2 ,pi/2)
collision_check = checkCollision(map_info(10,:), sample_node);

