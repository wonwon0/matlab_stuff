function [normale_effecteur]=PointToEffector_v1(pose_prox, pose_act, jacob_pt, jacob_eff )
    normale_effecteur=[0 0 0];
    vect_norm_plan=(pose_act-pose_prox)/norm(pose_act-pose_prox);
    K=jacob_pt/jacob_eff;
    for i=1:3
        K(i,:)=K(i,:)/norm(K(i,:))*vect_norm_plan(i);
        normale_effecteur=normale_effecteur+K(i,:);
    end
end