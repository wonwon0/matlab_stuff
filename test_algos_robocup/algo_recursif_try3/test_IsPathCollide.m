load('path.mat')
load('pose_obstacle.mat')
collision_bool = [];
for i = 2:size(path.points,1)-1
    path_temp.start = path.points(i,:);
    path_temp.goal = path.points(i+1,:);
    path_temp.points = [path_temp.start;path_temp.goal];
    collision_bool = [collision_bool; IsPathCollide( path_temp, 300, pose_obstacle )];
end
collision_bool