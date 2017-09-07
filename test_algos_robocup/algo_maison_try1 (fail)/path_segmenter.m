function [ new_target ] = path_segmenter( pose_robot, pose_target, pose_obstacle, gap_proxy, verbose )
    if isempty(pose_obstacle)
        new_target = pose_target;
        return
    end
    if verbose == 1
        display('step 2')
    end
    pose_obstacle_closest = pose_obstacle(1,:);
    dist_obstacle_closest = norm(pose_obstacle_closest - pose_robot);
    index = 1;
    for i=1:size(pose_obstacle,1)
        if dist_obstacle_closest> norm(pose_obstacle(i,:) - pose_robot);
            dist_obstacle_closest = norm(pose_obstacle(i,:) - pose_robot);
            pose_obstacle_closest = pose_obstacle(i,:);
            index = i ;
        end
    end
    
    
    dir = (pose_target-pose_robot)/norm(pose_target-pose_robot);
    vec_robot_2_obs = pose_obstacle_closest - pose_robot;
    len_along_path = vec_robot_2_obs * dir';
    collide = 1;
    facteur = 1;
    
    if sqrt(norm(vec_robot_2_obs)^2-len_along_path^2) < gap_proxy && len_along_path < norm(pose_target-pose_robot)
        vec_perp = (dir*len_along_path - pose_obstacle_closest) / norm(dir*len_along_path - pose_obstacle_closest);
        new_target_temp = pose_robot + dir*len_along_path + vec_perp* (gap_proxy - sqrt(norm(vec_robot_2_obs)^2-len_along_path^2));
        while collide == 1
            dist_obs_from_target = norm(pose_obstacle(1,:) - new_target_temp);
            for i=1:size(pose_obstacle,1)
                if dist_obs_from_target > norm(pose_obstacle(i,:) - new_target_temp);
                    dist_obs_from_target = norm(pose_obstacle(i,:) - new_target_temp);
                end
            end
            
            if dist_obs_from_target < gap_proxy
                facteur = -(facteur+0.5);
                new_target_temp = pose_robot + dir*len_along_path + vec_perp * (gap_proxy*facteur - sqrt(norm(vec_robot_2_obs)^2-len_along_path^2))
            else
                break
            end
        end
%         pose_obstacle(index,:) = [];
        new_target = [new_target_temp; path_segmenter( new_target_temp, pose_target, pose_obstacle, gap_proxy, 1 )]
        return
    else
        new_target = pose_target;
        return
    
    end


end

