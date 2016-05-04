% map info
mapUpBound = struct('start',[0; 100; 0],'end',[100; 100; 0]);
mapLeftBound = struct('start',[0; 0; 0],'end',[0; 100; 0]);
mapBottomBound = struct('start',[0; 0; 0],'end',[100; 0; 0]);
mapRightBound = struct('start',[100; 0; 0],'end',[100; 100; 0]);

% define the boundry of the objects
% top wall
topWallRight = struct('start',[45; 55; 0],'end',[45; 100; 0]);
topWallLeft = struct('start',[55; 55; 0],'end',[55; 100; 0]);
topWallBottom = struct('start',[45; 55; 0],'end',[55; 55; 0]);
% bottom wall
downWallRight = struct('start',[45; 0; 0],'end',[45; 45; 0]);
downWallLeft = struct('start',[55; 0; 0],'end',[55; 45; 0]);
downWallBottom = struct('start',[45; 45; 0],'end',[55; 45; 0]);

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