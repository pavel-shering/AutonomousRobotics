%% p - controller
% Input: 
% target - [x y] of the target point
% x - [x y theta] of the current robot pos
% prev_u  - [v theta_wheel] previous inputs
%        
% Output:
% u - [v theta_wheel] current input

function [u] = p_control(target, x, prev_u)
    p = 0.5;
    error_threshold = 0.1;

    theta_ref = atan2(target(2)-x(2), target(1)-x(1));
    theta = x(3);
    theta_error = theta_ref - theta;

    if (theta_error > pi)
        theta_error = theta_error - 2 * pi;
    elseif (theta_error < -pi)
        theta_error = theta_error + 2 * pi;
    end
    
    if (abs(target-x(1:2)) < error_threshold)
        u(1) = 0;
    else
        u(1) = prev_u(1);
    end

    u(2) = p * theta_error;
    if u(2) > pi/6
        u(2) = pi/6;
    elseif u(2) < -pi/6
        u(2) = - pi/6; 
    end
end
