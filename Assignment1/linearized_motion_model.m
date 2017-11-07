function [G] = linearized_motion_model(mu, input, dt)
% Outputs linearized motion model from three wheeled robot

if ~exist('dt', 'var')
    dt = 0.1;
end

G = [ 1, 0, ((input(1)*sin(mu(3)))/60 - (input(2)*sin(mu(3) + pi/3))/60 - (input(3)*sin(mu(3) - pi/3))/60)*dt
      0, 1, ((input(1)*cos(mu(3)))/60 - (input(2)*cos(mu(3) + pi/3))/60 - (input(3)*cos(mu(3) - pi/3))/60)*dt
      0, 0,                                                                 1];
  
  
% A = [ 1 0 -input(1)*sin(mu(3))*dt;
%       0 1 input(1)*cos(mu(3))*dt;
%       0 0 1];
end
