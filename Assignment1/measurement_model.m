function [ y ] = measurement_model( mup, input)
% Outputs measurement model from Mapping I slide 16 

y = [ input(1); input(2); input(3) - 9.7*pi/180 ];

end

