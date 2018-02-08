function [normale_effecteur]=PointToEffector(pose_prox, pose_act, jacob_pt, jacob_eff )
    normale_effecteur=[0 0 0 0 0 0];
    %if size(jacob_pt,2)==3
    if 1
        vect_norm_plan=[(pose_act-pose_prox)/norm(pose_act-pose_prox),0 ,0, 0];
        K=jacob_pt/jacob_eff;
        for i=1:6
            K(i,:)=K(i,:)/norm(K(i,:))*vect_norm_plan(i);
            normale_effecteur=normale_effecteur+K(i,:);
        end
        normale_effecteur(normale_effecteur<0.01)=0;
        normale_effecteur = normale_effecteur / norm(normale_effecteur);
    elseif size(jacob_pt,2)==2
        jacob_pt=[jacob_pt [0 0 0]'];
        vect_norm_plan=(pose_act-pose_prox)/norm(pose_act-pose_prox);
        K=jacob_pt/jacob_eff;
        for i=1:6
            K(i,:)=K(i,:)/norm(K(i,:))*vect_norm_plan(i);
            normale_effecteur=normale_effecteur+K(i,:);
        end
    end
end