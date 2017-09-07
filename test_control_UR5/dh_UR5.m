function [ dh ] = dh_UR5(link, ratio)
% link and ratio parameters allow to output variable dh parameters
% according to a point of interest located on one of the robot's links.

% links tells the link number on which the point is located on (starting 
% from 1 up to 6), and the ratio tells where the point is located on the
% link (from the base joint to the end joint of the link).

if nargin < 2
  link = 6;
  ratio = 1;
end

modifier = zeros(1,6)';
modifier(1:link) = 1;
modifier(link) = ratio;

%output les param dh du robot UR5
dh.a=[0 -425 -392.25 0 0 0]' .* modifier;
dh.b=[89.159 0 0 109.15 94.65 82.3]' .* modifier;
dh.alpha=[pi/2 0 0 pi/2 -pi/2 0]';
dh.theta=[0 -pi/2 0 -pi/2 0 0]';


end

