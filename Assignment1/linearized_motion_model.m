function [ G ] = linearized_motion_model(mup, input, dt)
% Outputs linearized motion model from three wheeled robot

if ~exist('dt', 'var')
    dt = 0.1;
end

% G = [ 1, 0, cos(mup(3))*((3^(1/2)*conj(input(2)))/16 - (3^(1/2)*conj(input(3)))/16) - sin(mup(3))*(conj(input(2))/24 - conj(input(1))/12 + conj(input(3))/24);
%       0, 1, cos(mup(3))*(conj(input(2))/24 - conj(input(1))/12 + conj(input(3))/24) + sin(mup(3))*((3^(1/2)*conj(input(2)))/16 - (3^(1/2)*conj(input(3)))/16);
%       0, 0,                                                                                                           1];

G = [ 1, 0, (input(1)*sin(mup(3)))/60 - (input(2)*sin(mup(3) - pi/3))/60 - (input(3)*sin(mup(3) + pi/3))/60
      0, 1, (input(1)*cos(mup(3)))/60 - (input(2)*cos(mup(3) - pi/3))/60 - (input(3)*cos(mup(3) + pi/3))/60
      0, 0,                                                                 1 ];  
end
