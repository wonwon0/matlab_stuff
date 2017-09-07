function path = FastPathPlaning_2(pose_obstacle,vit_obstacle , path, robot_speed, depth, avoid_dir, verbose)
    max_recurs = 8;
    gap_proxy = 300;
    [ collision_bool ] = IsPathCollide( path, gap_proxy, pose_obstacle );

    if collision_bool && depth < max_recurs
        
        [sub_target, avoid_dir] = SearchPoint_2( path, pose_obstacle, vit_obstacle, avoid_dir, gap_proxy, robot_speed);
        
        path_1 = GenerateSegment(path.start, sub_target);
        avoid_dir_1=avoid_dir;
        avoid_dir_2=avoid_dir;
        path_1 = FastPathPlaning_2(pose_obstacle,vit_obstacle , path_1, robot_speed, depth+1, avoid_dir_1, verbose);
        path_2 = GenerateSegment(sub_target, path.goal);
        path_2 = FastPathPlaning_2(pose_obstacle,vit_obstacle , path_2, robot_speed, depth+1, avoid_dir_2, verbose);
        path = JoinSegments(path_1, path_2);
    else
        if verbose == 1
            collision_bool;
        end
    end
end