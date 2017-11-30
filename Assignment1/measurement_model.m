function [ y ] = measurement_model( mup, input )
% Outputs measurement model for three wheeled robot

if ~exist('input', 'var')
    input = [1 1 1]';
end

y = [ mup(1); mup(2); (mup(3) - 9.7*pi/180 ) ];

end

