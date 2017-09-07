function [d_min, pose_prox, canard_dist]=verifDistance(limit,pose)

%verification proximité:
n=length(limit.limite);
d_min=0;
pose_prox=pose;

for i=1:n
    base=limit.limite(i).surfaces.surface1;
    top=limit.limite(i).surfaces.surface2;
    ratio_hauteure=(pose(3)-base(1,3))/(top(1,3)-base(1,3));
    section_inter=(1-ratio_hauteure)*base+(ratio_hauteure)*top;
    if limit.limite(i).rayonProxy>norm((pose-limit.limite(i).centroide))
        %si on est en haut ou en bas de l'objet:

        if (pose(3)<base(1,3));
            [d_min(i), x_d_min(i), y_d_min(i), ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), base(:,1) , base(:,2),true);
            if (dot([x_d_min(i) y_d_min(i)]-[limit.limite(i).centroide(1),limit.limite(i).centroide(2)],[pose(1),pose(2)]-[x_d_min(i) y_d_min(i)])<=0)
                %On est directement en dessous de l'obstacle
                pose_prox(i,:)=[pose(1) pose(2) base(1,3)];
                d_min(i)=sqrt((pose(3)-base(1,3))^2);
            else
                %On est en dessous et en biais de l'objet
                pose_prox(i,:)=[x_d_min(i) y_d_min(i) base(1,3)];
                d_min(i)=sqrt(d_min(i)^2+(pose(3)-base(1,3))^2);
            end

        elseif (pose(3)>top(1,3));
            [d_min(i), x_d_min(i), y_d_min(i), ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), top(:,1) , top(:,2),true);
            if (dot([x_d_min(i) y_d_min(i)]-[limit.limite(i).centroide(1),limit.limite(i).centroide(2)],[pose(1),pose(2)]-[x_d_min(i) y_d_min(i)])<=0)
                %On est directement au dessus de l'obstacle
                pose_prox(i,:)=[pose(1) pose(2) top(1,3)];
                d_min(i)=sqrt((pose(3)-top(1,3))^2);
            else
                %On est directement au dessus et en biais de l'obstacle
                pose_prox(i,:)=[x_d_min(i) y_d_min(i) top(1,3)];
                d_min(i)=sqrt(d_min(i)^2+(pose(3)-top(1,3))^2);
            end

        end
        if pose(3)>base(1,3) && pose(3)<top(1,3)
            %si on est entre le plan de dessus de l'objet et le plan de dessous
            [d_min(i), x_d_min(i), y_d_min(i), ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), section_inter(:,1) , section_inter(:,2),true);
            if (dot([x_d_min(i) y_d_min(i)]-[limit.limite(i).centroide(1),limit.limite(i).centroide(2)],[pose(1),pose(2)]-[x_d_min(i) y_d_min(i)])<=0)
                %On est dans l'objet
                pose_prox(i,:)=pose;
            else
                pose_prox(i,:)=[x_d_min(i) y_d_min(i) pose(3)];
            end
        end

    else
        d_min(i)=10;
        pose_prox(i,:)=pose;
    end

end

    

