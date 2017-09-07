function [ bool ] = VerifySub_target( pose_obstacle, sub_target, gap_proxy )
    bool=0;
    for i = 1:size(pose_obstacle,1)
        dist_sub_2_obs = norm(pose_obstacle(i,:)-sub_target);
        if dist_sub_2_obs < gap_proxy
            bool = 1;
            break
        end
    end

end

