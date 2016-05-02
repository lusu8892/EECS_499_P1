% map info
mapUpBound = struct('start',[-50; 50; 0],'end',[50; 50; 0]);
mapLeftBound = struct('start',[50; 50; 0],'end',[50; -50; 0]);
mapBottomBound = struct('start',[50; -50; 0],'end',[-50; -50; 0]);
mapRightBound = struct('start',[-50; -50; 0],'end',[-50; 50; 0]);

% define the boundry of the objects
% top wall
topWallRight = struct('start',[-5; 5; 0],'end',[-5; 50; 0]);
topWallLeft = struct('start',[5; 5; 0],'end',[5; 50; 0]);
topWallBottom = struct('start',[-5; 5; 0],'end',[5; 5; 0]);
% bottom wall
downWallRight = struct('start',[-5; -5; 0],'end',[-5; -50; 0]);
downWallLeft = struct('start',[5; -5; 0],'end',[5; -50; 0]);
downWallBottom = struct('start',[-5; -5; 0],'end',[5; -5; 0]);

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