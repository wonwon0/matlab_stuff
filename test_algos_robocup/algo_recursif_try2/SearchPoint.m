function [ sub_target, avoid_dir ] = SearchPoint( path, pose_obstacle, avoid_dir , gap_proxy)
%détecte si un obstacle est dans le chemin du path (composé de deux points)

    res = 20;
    pose_robot = path.start;
    pose_target = path.goal;

    [pose_obstacle_closest, ~ ] = FindClosestObstacle( pose_target, path, pose_obstacle, gap_proxy);
    if isnan(pose_obstacle_closest)
        sub_target = pose_target;
        return
    end
    dir = (pose_target-pose_robot)/norm(pose_target-pose_robot);
    vec_robot_2_obs = pose_obstacle_closest - pose_robot;
    len_along_path = vec_robot_2_obs * dir';
    
    if len_along_path > 0 && len_along_path < norm(pose_target-pose_robot)
        %vec_perp = (dir*len_along_path - pose_obstacle_closest) / norm(dir*len_along_path - pose_obstacle_closest);
        vec_perp = cross([dir, 0],[0, 0, 1]);
        
        vec_perp = [vec_perp(1), vec_perp(2)];

        if isnan(avoid_dir)
            sub_target_1 = pose_robot + dir*len_along_path + vec_perp * res;
            sub_target_2 = pose_robot + dir*len_along_path - vec_perp * res;

            bool_sub_target_1 = VerifySub_target( pose_obstacle, sub_target_1, gap_proxy );
            bool_sub_target_2 = VerifySub_target( pose_obstacle, sub_target_2, gap_proxy );
            while bool_sub_target_1

                sub_target_1 = sub_target_1 + vec_perp * res;
                bool_sub_target_1 = VerifySub_target( pose_obstacle, sub_target_1, gap_proxy );

            end
            sub_target_1 = sub_target_1 + vec_perp * 0.01 *res;
            while bool_sub_target_2

                sub_target_2 = sub_target_2 - vec_perp * res;
                bool_sub_target_2 = VerifySub_target( pose_obstacle, sub_target_2, gap_proxy );
                %[ pose_obstacle_closest_2 , dist_sub_target_2_obs_2 ] = FindClosestObstacle( sub_target_2, path, pose_obstacle, gap_proxy);

            end
            sub_target_2 = sub_target_2 - vec_perp * 0.01 *res;
            if abs(norm(path.start - sub_target_1)- norm(path.start - sub_target_2)) < 600
                sub_targetttt_1 = sub_target_1;
                sub_target = sub_target_1;
                avoid_dir = -vec_perp;
            else
                if norm(path.start - sub_target_1) < norm(path.start - sub_target_2)
                    sub_targetttt_1 = sub_target_1;
                    sub_target = sub_target_1;
                    avoid_dir = -vec_perp;
                else
                    sub_targetttt_2 = sub_target_2;
                    sub_target = sub_target_2;
                    avoid_dir = vec_perp;
                end
            end
%             line([path.start(1)+len_along_path*dir(1), path.start(1)+len_along_path*dir(1)+avoid_dir(1)*1000], [path.start(2)+len_along_path*dir(2), path.start(2)+len_along_path*dir(2)+avoid_dir(2)*1000], 'LineWidth',3)
%             hold on
        else
            if avoid_dir * vec_perp' < 0
                vec_perp = -vec_perp;
            end
            sub_target = pose_robot + dir*len_along_path + vec_perp * res;
            bool_sub_target = VerifySub_target( pose_obstacle, sub_target, gap_proxy );
            while bool_sub_target

                sub_target = sub_target - vec_perp * res;
                bool_sub_target = VerifySub_target( pose_obstacle, sub_target, gap_proxy );

            end
            sub_target = sub_target - vec_perp * 0.01*res;
            avoid_dir=vec_perp;
%             line([path.start(1)+len_along_path*dir(1), path.start(1)+len_along_path*dir(1)+vec_perp(1)*1000], [path.start(2)+len_along_path*dir(2), path.start(2)+len_along_path*dir(2)+vec_perp(2)*1000], 'LineWidth',3)
%             hold on
        end
        
        
        %sub_target = FindClosestPoint( pose_robot, sub_target_1, sub_target_2);
    else
        sub_target = pose_target;
    end

end





