function [ y ] = measurement_model( mup, input)
% Outputs measurement model from Mapping I slide 16 

y = [ mup(1); mup(2); mup(3) - 99.7* pi / 180 ];

end

