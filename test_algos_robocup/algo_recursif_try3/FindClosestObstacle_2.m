function [closest_obs, vit_closest_obs, dist_point_obs] = FindClosestObstacle_2( point, path, pose_obstacle, vit_obstacle, gap_proxy)
    pose_obstacle_col = [];
    vit_obstacle_col = [];
    for i = 1:size(pose_obstacle,1)
        if IsPathCollide( GenerateSegment( path.start, point ), gap_proxy, pose_obstacle(i,:))
            pose_obstacle_col = [pose_obstacle_col ; pose_obstacle(i,:)];
            vit_obstacle_col = [vit_obstacle_col; vit_obstacle(i,:)];
        end
    end
    if isempty(pose_obstacle_col)
        closest_obs = NaN;
        dist_point_obs = NaN;
        return
    end
    dist_point_obs = norm(path.start - pose_obstacle_col(1,:));
    closest_obs = pose_obstacle_col(1,:);
    vit_closest_obs = vit_obstacle_col(1, :);
    for i = 1:size(pose_obstacle_col,1)
        if norm(path.start - pose_obstacle_col(i,:)) < dist_point_obs
            dist_point_obs = norm(path.start - pose_obstacle_col(i,:));
            closest_obs = pose_obstacle_col(i,:);
            vit_closest_obs = vit_obstacle_col(i,:);
        end
    end
end
