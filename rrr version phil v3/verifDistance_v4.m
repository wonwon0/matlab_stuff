function [d_min, pose_prox, pose_prox_pt_act]=verifDistance_v4(limit,pose,L, L_eff,theta)

%verification proximit�:
n=length(limit.limite);

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
            prod_scal=((top-base)*(pose-base)')/norm(top-base)^2;
            %si on est en haut ou en bas de l'objet:
            if prod_scal<0 || prod_scal>1;
                if prod_scal>1 %on est en haut
                    point_long=limit.limite(t).surfaces.base+prod_scal*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                    if norm(pose-point_long)< limit.limite(t).surfaces.dia/2 % on est direct au dessus du cylindre
                        (prod_scal-1)*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                        pose_prox(t,:)=pose-(prod_scal-1)*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                        d_min(t)=norm(pose_prox(t,:)-pose);
                    else % on est au dessus mais en biais du cylindre
                        point_long=point_long+limit.limite(t).surfaces.dia/2*(pose-point_long)/norm(pose-point_long);
                        pose_prox(t,:)=point_long-(prod_scal-1)*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                        d_min(t)=norm(pose_prox(t,:)-pose);
                    end
                else
                     point_long=limit.limite(t).surfaces.base+prod_scal*limit.limite(t).surfaces.axe;
                    if norm(pose-point_long)< limit.limite(t).surfaces.dia/2 % on est direct au dessous du cylindre
                        pose_prox(t,:)=pose-(prod_scal)*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                        d_min(t)=norm(pose_prox(t,:)-pose);
                    else % on est au dessous mais en biais du cylindre
                        point_long=point_long+limit.limite(t).surfaces.dia/2*(pose-point_long)/norm(pose-point_long);
                        pose_prox(t,:)=point_long-(prod_scal-1)*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                        d_min(t)=norm(pose_prox(t,:)-pose);
                    end
                end
            else
                point_long=limit.limite(t).surfaces.base+prod_scal*limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
                pose_prox(t,:)=point_long+limit.limite(t).surfaces.dia/2*(pose-point_long)/norm(pose-point_long);
                d_min(t)=norm(pose_prox(t,:)-pose);

%                 if norm(pose_prox(t,:)-pose)>norm(p_A-pose)
%                     d_min(t)=-d_min(t);
%                 end
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
if L~=L_eff
    n=size(pose_prox,1);
    for t=1:n
        vec_norm(t,:)=(pose-pose_prox(t,:))/norm(pose-pose_prox(t,:));
    end
    normale_effecteur=PointToEffector_v4(vec_norm, theta, L_eff,L );
    for t=1:n
        pose_prox(t,:)=pose+normale_effecteur;
    end
end