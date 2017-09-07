function [ collision_bool ] = IsPathCollide( path, gap_proxy, pose_obstacle )
    
    collision_bool = 0;

    if isempty(pose_obstacle)
        return
    end
    if norm(path.goal-path.start) ==0
        return
    end
    dir = (path.goal-path.start)/norm(path.goal-path.start);
    
    for i = 1:size(pose_obstacle,1)
        vec_robot_2_obs_temp = pose_obstacle(i,:) - path.start;
        len_along_path_temp = vec_robot_2_obs_temp * dir';
        dist_from_path_temp = sqrt(norm(vec_robot_2_obs_temp)^2-len_along_path_temp^2);
        if gap_proxy > dist_from_path_temp && len_along_path_temp > 0
            collision_bool = 1;
            break
        end
    end


end

