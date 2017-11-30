function [ H ] = linearized_measurement_model( mup, input )
% Outputs measurement model for three wheeled robot
if ~exist('input', 'var')
    input = [1 1 1]';
end

H = eye(3);

end
