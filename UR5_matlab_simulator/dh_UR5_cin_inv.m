function [ dh ] = dh_UR5_cin_inv(end_effector_lenght)
% link and ratio parameters allow to output variable dh parameters
% according to a point of interest located on one of the robot's links.

% links tells the link number on which the point is located on (starting 
% from 1 up to 6), and the ratio tells where the point is located on the
% link (from the base joint to the end joint of the link).

%output les param dh du robot UR5

dh.a=[0 -425 -392.25 0 0 0]';
dh.b=[89.159 0 0 109.15 94.65 end_effector_lenght]';
dh.alpha=[pi/2 0 0 pi/2 -pi/2 0]';
dh.theta=[0 -pi/2 0 -pi/2 0 0]';


end

