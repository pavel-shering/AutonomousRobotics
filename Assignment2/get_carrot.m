%% distanceToCarrot
% A function to calculate the location of the carrot and determine 
% whether the carrot is past the given line segment 
%
% Input: 
% p1 - beginning point of the line segment 
% p2 - end point of the line segment
% x  - position of the vehicle
%        
% Output:
% carrot_outside - A boolean of whether the carrot is past the end of the
%                  desired destination
%
% carrot_point   - A coordinate value of the carrot the robot is following


function [carrot_outside carrot_point] = get_carrot(p1, p2, x, r)
    carrot_outside = 0;
    line_segment = p2 - p1;
    p1_to_x = x - p1;

    % Compute the distance to the line as a vector, using the projection
    projection = p1 + (dot(line_segment,p1_to_x) / norm(line_segment)^2) * line_segment;
    carrot_point = projection + r * line_segment ./ norm(line_segment);

    if (norm(carrot_point - p1) > norm(line_segment))
        carrot_outside = 1;
    end
end