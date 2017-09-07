function [ bool ] = VerifySub_target_2( pose_robot, pose_obstacle, sub_target, gap_proxy )
    bool=0;
    path_temp.start = pose_robot;
    path_temp.goal = sub_target;
    path_temp.points = [path_temp.start;path_temp.goal];
    
    for i = 1:size(pose_obstacle,1)
        if IsPathCollide(path_temp, gap_proxy, pose_obstacle(i,:))
            bool = 1;
            break
        end
    end
end