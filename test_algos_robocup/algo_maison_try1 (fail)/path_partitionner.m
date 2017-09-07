clear
close all

pose_robot = [0, 0];
pose_target = [1000, 1000];
pose_obstacle = [750, 500; 420 510; 660 740];
rayon_robot = 150;
gap_proxy = 320;
new_target = [];
new_targets = [new_target; path_segmenter( pose_robot, pose_target, pose_obstacle, gap_proxy ,0)];








datas = [pose_robot;pose_target; pose_obstacle; new_target];
plot(datas(:,1), datas(:,2), 'Xb')
hold on
line([pose_robot(1), pose_target(1)], [pose_robot(2), pose_target(2)]);
hold on
plot(new_targets(:,1), new_targets(:,2),'Or')
hold off