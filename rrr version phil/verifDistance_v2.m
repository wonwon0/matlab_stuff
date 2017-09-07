function [d_mins, poses_prox]=verifDistance_v2(limit,pose)

%verification proximité:
n=length(limit.limite);

for j=1:size(pose,1)
    for i=1:n
        base=limit.limite(i).surfaces.surface1;
        top=limit.limite(i).surfaces.surface2;
        if limit.limite(i).rayonProxy>norm((pose(j,:)-limit.limite(i).centroide))
            [d_min_p, x_d_min, y_d_min, ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(j,1), pose(j,2), base(:,1) , base(:,2),true);
            if (pose(j,3)<base(1,3));
                if (dot([x_d_min y_d_min]-[limit.limite(i).centroide(1),limit.limite(i).centroide(2)],[pose(j,1),pose(j,2)]-[x_d_min y_d_min])<=0)
                    %On est directement en dessous de l'obstacle
                    pose_prox=[pose(j,1) pose(j,2) base(1,3)];
                    d_min=sqrt((pose(j,3)-base(1,3))^2);
                else
                    %On est en dessous et en biais de l'objet
                    pose_prox=[x_d_min y_d_min base(1,3)];
                    d_min=sqrt(d_min_p^2+(pose(j,3)-base(1,3))^2);
                end
            elseif (pose(j,3)>top(1,3));
                if (dot([x_d_min y_d_min]-[limit.limite(i).centroide(1),limit.limite(i).centroide(2)],[pose(j,1),pose(j,2)]-[x_d_min y_d_min])<=0)
                    %On est directement au dessus de l'obstacle
                    pose_prox=[pose(j,1) pose(j,2) top(1,3)];
                    d_min=sqrt((pose(j,3)-top(1,3))^2);
                else
                    %On est directement au dessus et en biais de l'obstacle
                    pose_prox=[x_d_min y_d_min top(1,3)];
                    d_min=sqrt(d_min_p^2+(pose(j,3)-top(1,3))^2);
                end
            elseif (dot([x_d_min y_d_min]-[limit.limite(i).centroide(1),limit.limite(i).centroide(2)],[pose(j,1),pose(j,2)]-[x_d_min y_d_min])<=0)
                %On est dans l'objet
                pose_prox=pose(j,:);
                d_min=-1;

            else
                %On est à coter de l'objet
                pose_prox=[x_d_min y_d_min pose(j,3)];
                d_min=d_min_p;
            end
        else
            d_min=10;
            pose_prox=pose(j,:);
        end
        d_mins(j*(i-1)+j)=d_min;
        poses_prox(j*(i-1)+j,:)=pose_prox;
    end
end
