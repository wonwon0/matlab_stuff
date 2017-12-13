function [d_min, pose_prox, pose_prox_pt_act]=verifDistance_v5(limit,pose,jacobian, jacobian_eff,theta)

%verification proximit�:
n=length(limit.limite);
pose = pose(1:3);
for t=1:n
    if all(limit.limite(t).type=='poly')
        base=limit.limite(t).surfaces.surface1;
        top=limit.limite(t).surfaces.surface2;
        ratio_hauteure=(pose(3)-base(1,3))/(top(1,3)-base(1,3));
        section_inter=(1-ratio_hauteure)*base+(ratio_hauteure)*top;
        if limit.limite(t).rayonProxy>norm((pose-limit.limite(t).centroide))
            %si on est en haut ou en bas de l'objet:
            if (pose(3)<base(1,3));
                [d_min(t), x_d_min(t), y_d_min(t), ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), base(:,1) , base(:,2),true);
                if (dot([x_d_min(t) y_d_min(t)]-[limit.limite(t).centroide(1),limit.limite(t).centroide(2)],[pose(1),pose(2)]-[x_d_min(t) y_d_min(t)])<=0)
                    %On est directement en dessous de l'obstacle
                    pose_prox(t,:)=[pose(1) pose(2) base(1,3)];
                    d_min(t)=sqrt((pose(3)-base(1,3))^2);
                else
                    %On est en dessous et en biais de l'objet
                    pose_prox(t,:)=[x_d_min(t) y_d_min(t) base(1,3)];
                    d_min(t)=sqrt(d_min(t)^2+(pose(3)-base(1,3))^2);
                end
            elseif (pose(3)>top(1,3));
                [d_min(t), x_d_min(t), y_d_min(t), ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), top(:,1) , top(:,2),true);
                if (dot([x_d_min(t) y_d_min(t)]-[limit.limite(t).centroide(1),limit.limite(t).centroide(2)],[pose(1),pose(2)]-[x_d_min(t) y_d_min(t)])<=0)
                    %On est directement au dessus de l'obstacle
                    pose_prox(t,:)=[pose(1) pose(2) top(1,3)];
                    d_min(t)=sqrt((pose(3)-top(1,3))^2);
                else
                    %On est directement au dessus et en biais de l'obstacle
                    pose_prox(t,:)=[x_d_min(t) y_d_min(t) top(1,3)];
                    d_min(t)=sqrt(d_min(t)^2+(pose(3)-top(1,3))^2);
                end
            end
            if pose(3)>base(1,3) && pose(3)<top(1,3)
                %si on est entre le plan de dessus de l'objet et le plan de dessous
                [d_min(t), x_d_min(t), y_d_min(t), ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), section_inter(:,1) , section_inter(:,2),true);
                if (dot([x_d_min(t) y_d_min(t)]-[limit.limite(t).centroide(1),limit.limite(t).centroide(2)],[pose(1),pose(2)]-[x_d_min(t) y_d_min(t)])<=0)
                    %On est dans l'objet
                    pose_prox(t,:)=pose;
                else
                    pose_prox(t,:)=[x_d_min(t) y_d_min(t) pose(3)];
                end
            end
        else
            d_min(t)=10;
            pose_prox(t,:)=pose;
        end
    elseif all(limit.limite(t).type=='tube')
        if limit.limite(t).rayonProxy>norm((pose-limit.limite(t).centroide))
            base=limit.limite(t).surfaces.base;
            top=limit.limite(t).surfaces.base+limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
            pos = linePosition3d(pose, [base top]);
            %si on est en haut ou en bas de l'objet:
            if pos<0 || pos>1;
                if pos>1
                    p_A=(pos*limit.limite(t).surfaces.longueur)*limit.limite(t).surfaces.axe+base;
                    p_B=(pose-p_A)*limit.limite(t).surfaces.dia/2/norm(pose-p_A)-pos*limit.limite(t).surfaces.axe;
                    pose_prox(t,:)=p_B+top;
                    d_min(t)=norm(pose_prox(t,:)-pose);
                else
                    p_A=(pos*limit.limite(t).surfaces.longueur)*limit.limite(t).surfaces.axe+base;
                    p_B=(pose-p_A)*limit.limite(t).surfaces.dia/2/norm(pose-p_A)-pos*limit.limite(t).surfaces.axe;
                    pose_prox(t,:)=p_B+base;
                    d_min(t)=norm(pose_prox(t,:)-pose);
                end
            else 
                p_A=(pos*limit.limite(t).surfaces.longueur)*limit.limite(t).surfaces.axe+base;
                p_B=p_A+(pose-p_A)*limit.limite(t).surfaces.dia/2/norm(pose-p_A);
                pose_prox(t,:)=p_B;
                d_min(t)=norm(pose_prox(t,:)-pose);
                if norm(pose_prox(t,:)-pose)>norm(p_A-pose)
                    d_min(t)=-d_min(t);
                end
            end
        else
            d_min(t)=10;
            pose_prox(t,:)=pose;
        end
    elseif all(limit.limite(t).type=='sphe')
        if limit.limite(t).rayonProxy>norm((pose-limit.limite(t).centroide))
            pose_prox(t,:)=pose+(limit.limite(t).radius-norm((pose-limit.limite(t).centroide)))*(pose-limit.limite(t).centroide)/norm(pose-limit.limite(t).centroide);
            d_min(t)=abs((limit.limite(t).radius-norm((pose-limit.limite(t).centroide))));
        else
            d_min(t)=10;
            pose_prox(t,:)=pose;
        end
    end
end
pose_prox_pt_act=pose_prox;
if jacobian~=jacobian_eff
    n=size(pose_prox,1);
    for t=1:n
        vec_norm(t,:)=(pose-pose_prox(t,:))/norm(pose-pose_prox(t,:));
    end
    normale_effecteur=PointToEffector(vec_norm, theta, jacobian_eff,jacobian );
    for t=1:n
        pose_prox(t,:)=pose+normale_effecteur;
    end
end