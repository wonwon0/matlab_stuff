function path = FastPathPlaning(pose_obstacle, path, depth, avoid_dir, verbose)
    max_recurs = 4;
    gap_proxy = 400;
    [ collision_bool ] = IsPathCollide( path, gap_proxy, pose_obstacle );

    if collision_bool && depth < max_recurs
        
        [sub_target, avoid_dir] = SearchPoint( path, pose_obstacle , avoid_dir, gap_proxy);
        
        path_1 = GenerateSegment(path.start, sub_target);
        avoid_dir_1=avoid_dir;
        avoid_dir_2=avoid_dir;
        path_1 = FastPathPlaning(pose_obstacle, path_1, depth+1, avoid_dir_1, verbose);
        path_2 = GenerateSegment(sub_target, path.goal);
        path_2 = FastPathPlaning(pose_obstacle, path_2, depth+1, avoid_dir_2, verbose);
        path = JoinSegments(path_1, path_2);
    else
        if verbose == 1
            collision_bool
        end
    end
end