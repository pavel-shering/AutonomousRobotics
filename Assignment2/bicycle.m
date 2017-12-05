%% Bicycle robot model kinematics.
%
% Input: 
% xprev = [x, y, theta]' 
% v     - speed
% delta - steering angle
% l     - frame size, distance from front wheel to rear
% dt    - time step
%        
% Output:
% xcur - new robot state [x, y, theta]'

function xcur = bicycle(xprev,v,delta,l,dt)

    % Motion increment in the body frame
    dx_b = dt*[v 0 v*tan(delta)/l]';

    % Rotation matrix for conversion from body to inertial frame
    R = rot(-xprev(3),3);

    % Robot state update in inertial frame
    xcur = xprev + R*dx_b;
    xcur(3) = mod(xcur(3),2*pi);
end 

