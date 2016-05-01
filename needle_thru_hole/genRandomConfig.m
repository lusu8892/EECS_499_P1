%% genRandomConfig.m
% This function is used to check if needle semicircle curve intersected
% with the straight line e.g., the wall or object
% Input: 1. the straight to check
%        2. the needle struc
%		 3. the transformation matrix from needle to tissue
% 
% Output: boolean value which represents collided or not.
%         false: no intersection with semicircle curve
%         true: intersected with semicircle curve

%% main function
function [ sample_node ] = genRandomConfig(rnd_num)
    sample_node.position = [];
    sample_node.direction = 0;
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
    
    % needle structure
    structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);
    % define a struture to store transformation matrix
    trans_mat = struct('rot', zeros(3), 'trans', zeros(3,1));
    
    while (true)
       
        % based on map info generate random sample configuration and check collision
        x = (mapLeftBound.start(1) - mapRightBound.start(1)) * rnd_num;
        y = (mapUpBound.start(2) - mapBottomBound.start(2)) * rnd_num;
        z = 0;
        theta = (2 * pi) * rnd_num;
        trans_mat.rot = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
        trans_mat.trans = [x;y;z];
        
        check_result_list = [];
        for i = 1 : length(map_info)
            check_result_list(i,:) = checkCollision(map_info(i), structNeedleGeometry, trans_mat);
            if (check_result_list(i,:) == true)
                break;
            else
                continue;
            end
        end
        
        if (isequal(check_result_list, zeros(length(map_info),1)))
            sample_node.position = [x;y;z];
            sample_node.direction = theta;
            return;
        else
            continue;
        end
    end
end