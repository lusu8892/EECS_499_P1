%% circleDrawing.m
% This is the function to draw circular curve
% Input:
%       1. x, y is the center
%       2. r is circular shape radius
%       3. theta_1 is the starting angle
%       4. theta_2 is the ending angle
% Output:
%       1. drawing
function circleDrawing(x,y,r,theta_1,theta_2)
    hold on

    th = theta_1:pi/50:theta_2;

    xunit = r * cos(th) + x;

    yunit = r * sin(th) + y;

    plot(xunit, yunit, 'color', 'black', 'LineWidth', 2);
end

