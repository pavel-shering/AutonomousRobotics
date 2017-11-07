function [ H ] = linearized_measurement_model( mup, input)
% Outputs measurement model from Mapping I slide 16 

H = [ mup(1) 0 0; 0 mup(2) 0; 0 0 mup(3)];

end
