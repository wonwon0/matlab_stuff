function [closest_point] = FindClosestPoint( pose_ref, point_1, point_2)
    dist_pose_ref_point_1 = norm(point_1 - pose_ref);
    dist_pose_ref_point_2 = norm(point_2 - pose_ref);
    if dist_pose_ref_point_1 < dist_pose_ref_point_2
        closest_point = point_1;
    else
        closest_point = point_2;
    end
end
