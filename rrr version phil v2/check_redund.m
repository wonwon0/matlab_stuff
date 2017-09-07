function [ vec_norm ] = check_redund(pose_act, vec_norm_red,d_min)
    %fonction détectant les vecteuyrs normaux redondants. Si deux points
    %proximaux sont froment un vecteur qui est perpendiculaire à une des
    %deux normales attachées à ces points, un des deux points forme alors
    %une normale à l'effecteur redondante. Ceci cause un problème lorsque
    %on veut slider sur des joints.
    remove=[];
    vec_norm=unique(vec_norm_red,'rows');
    vec_norm(~any(isnan(vec_norm),2),:);
    n=size(vec_norm,1);
    for i=1:n
        for j=i+1:n
            point_1=pose_act+vec_norm(i,:)*d_min(i);
            point_2=pose_act+vec_norm(j,:)*d_min(j);
            vec_12=(point_2-point_1)/norm(point_2-point_1);
            if any(isnan(vec_12))
                dbstop at 15 in check_redund;
            end
            a=norm(cross(vec_12,vec_norm(i,:)));
            b=norm(cross(vec_12,vec_norm(j,:)));
            if a>0.9999
                remove=[remove j];
            end
        end
        
        vec_norm(remove,:)=[];
        remove=[];
        n=size(vec_norm,1);
    end
end

