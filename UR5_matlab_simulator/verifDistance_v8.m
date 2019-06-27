function [d_min, poses_prox, poses_points, normales_effecteur]=verifDistance_v8(limit,poses,jacobians, jacobian_eff, pose_eff)

jacobians_base = {};
poses_prox_base = zeros(0,3);
d_min_base = zeros(0,1);
jacobians_top = {};
poses_prox_top = zeros(0,3);
d_min_top = zeros(0,1);
jacobians_side = {};
poses_prox_side = zeros(0,3);
d_min_side = zeros(0,1);

%verification proximit�:
n=length(limit.limite);
pose = poses(1:3);
poses_prox = zeros(0,3);
d_min = zeros(0,1);
jacobians_prox = {};
poses_points = zeros(0,3);
for t=1:n
    is_poses_far = limit.limite(t).rayonProxy<vecnorm((poses-limit.limite(t).centroide'));
    is_collision_filter = limit.limite(t).collision_filter;
    limits_to_consider = ~is_poses_far & is_collision_filter;
    
    
    poses_to_limit = poses(:, limits_to_consider);
    jacobians_to_limit = jacobians(limits_to_consider);
    if isempty(poses_to_limit)
        continue
    end
    if strcmp(limit.limite(t).type,'poly')
        base=limit.limite(t).surfaces.surface1;
        top=limit.limite(t).surfaces.surface2;
        ratio_hauteure=(pose(3)-base(1,3))/(top(1,3)-base(1,3));
        section_inter=(1-ratio_hauteure)*base+(ratio_hauteure)*top;

        %si on est en haut ou en bas de l'objet:
        poses_base = poses_to_limit(:, poses_to_limit(3,:)<base(1,3));
        jacobians_base = jacobians_to_limit(poses_to_limit(3,:)<base(1,3));
        poses_top = poses_to_limit(:, poses_to_limit(3,:)>top(1,3));
        jacobians_top = jacobians_to_limit(poses_to_limit(3,:)>top(1,3));
        poses_side = poses_to_limit(:, (poses_to_limit(3,:)>top(1,3)) == (poses_to_limit(3,:)<base(1,3)));
        jacobians_side = jacobians_to_limit((poses_to_limit(3,:)>top(1,3)) == (poses_to_limit(3,:)<base(1,3)));

        if ~isempty(poses_base)
            [d_min_base, x_d_min_base, y_d_min_base, ~, ~, ~, ~, ~, ~, ~] = p_poly_dist(poses_base(1,:), poses_base(2,:), base(:,1) , base(:,2),true);
            
            %poses directement en dessous de l'obstacle
            is_under = (dot([x_d_min_base y_d_min_base]-[limit.limite(t).centroide(1),limit.limite(t).centroide(2)],[poses_base(1,:)',poses_base(2,:)']-[x_d_min_base y_d_min_base],2)<=0);
            poses_base_under = poses_base(:, is_under);
            
            if ~isempty(poses_base_under)
                jacobians_base_under = jacobians_base(is_under);
                poses_prox_base_under = [poses_base_under(1,:)' poses_base_under(2,:)' ones(sum(is_under),1)*base(1,3)];
                d_min_base_under = abs(poses_base_under(3,:)'- base(1,3));
            else
                jacobians_base_under = {};
                poses_prox_base_under = zeros(0,3);
                d_min_base_under = zeros(0,1);
            end
            
            %poses en dessous et en biais de l'objet
            poses_base_not_under = poses_base(:, ~is_under);
            x_d_min_base_not_under = x_d_min_base(~is_under);
            y_d_min_base_not_under = y_d_min_base(~is_under);
            if ~isempty(poses_base_not_under)
                jacobians_base_not_under = jacobians_base(~is_under);
                poses_prox_base_not_under = [x_d_min_base_not_under y_d_min_base_not_under ones(sum(~is_under),1)*base(1,3)];
                d_min_base_not_under = sqrt(d_min_base(~is_under).^2 + (poses_base_not_under(3,:)' - base(1,3)).^2);
            else
                jacobians_base_not_under = {};
                poses_prox_base_not_under = zeros(0,3);
                d_min_base_not_under = zeros(0,1);
            end
            poses_base = [poses_base_under';poses_base_not_under'];
            poses_prox_base = [poses_prox_base_under ; poses_prox_base_not_under];
            d_min_base = [d_min_base_under; d_min_base_not_under];
            jacobians_base = [jacobians_base_under, jacobians_base_not_under];
        else
            poses_base = zeros(0,3);
        end
        if ~isempty(poses_top)
            [d_min_top, x_d_min_top, y_d_min_top, ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(poses_top(1,:), poses_top(2,:), top(:,1) , top(:,2),true);
            
            %poses directement au dessus de l'obstacle
            is_over = (dot([x_d_min_top y_d_min_top]-[limit.limite(t).centroide(1),limit.limite(t).centroide(2)],[poses_top(1,:)',poses_top(2,:)']-[x_d_min_top y_d_min_top],2)<=0);
            poses_top_over = poses_top(:,is_over);
            if ~isempty(poses_top_over)
                jacobians_top_over = jacobians_top(is_over);
                poses_prox_top_over = [poses_top_over(1,:)' poses_top_over(2,:)' ones(sum(is_over),1)*top(1,3)];
                d_min_top_over = abs(poses_top_over(3,:)'- top(1,3));
            else
                jacobians_top_over = {};
                poses_prox_top_over = zeros(0,3);
                d_min_top_over = zeros(0,1);
            end
            
            
            %poses au dessus et en biais de l'objet
            poses_top_not_over = poses_top(:, ~is_over);
            x_d_min_top_not_over = x_d_min_top(~is_over);
            y_d_min_top_not_over = y_d_min_top(~is_over);
            
            if ~isempty(poses_top_not_over)
                jacobians_top_not_over = jacobians_top(~is_over);
                poses_prox_top_not_over = [x_d_min_top_not_over y_d_min_top_not_over ones(sum(~is_over),1)*top(1,3)];
                d_min_top_not_over = sqrt(d_min_top(~is_over).^2 + (poses_top_not_over(3,:)' - top(1,3)).^2);
            else
                jacobians_top_not_over = {};
                poses_prox_top_not_over = zeros(0,3);
                d_min_top_not_over = zeros(0,1);
            end
            
            poses_top = [poses_top_over';poses_top_not_over'];
            poses_prox_top = [poses_prox_top_over ; poses_prox_top_not_over];
            d_min_top = [d_min_top_over; d_min_top_not_over];
            jacobians_top = [jacobians_top_over, jacobians_top_not_over];
        else
            poses_top = zeros(0,3);
        end
        if ~isempty(poses_side)
            [d_min_side, x_d_min_side, y_d_min_side, ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(poses_side(1,:), poses_side(2,:), section_inter(:,1) , section_inter(:,2),true);
            
            % on est dans l'objet
            poses_side_inside = poses_side(:, d_min_side<0);
            d_min_side_inside = d_min_side(d_min_side<0);
            poses_prox_side_inside = poses_side(:,d_min_side<0)';
            jacobians_side_inside = jacobians_side(d_min_side<0);
            
            
            %on est a cote de l'objet
            d_min_side_outside = d_min_side(d_min_side>=0);
            poses_side_outside = poses_side(:, d_min_side>=0);
            x_d_min_side_outside = x_d_min_side(d_min_side>=0);
            y_d_min_side_outside = y_d_min_side(d_min_side>=0);
            jacobians_side_outside = jacobians_side(d_min_side>=0);
            poses_prox_side_outside = [x_d_min_side_outside y_d_min_side_outside poses_side_outside(3,:)'];
            
            poses_side = [poses_side_inside'; poses_side_outside'];
            poses_prox_side = [poses_prox_side_inside; poses_prox_side_outside];
            d_min_side = [d_min_side_inside; d_min_side_outside];
            jacobians_side = [jacobians_side_inside, jacobians_side_outside];
        else
            poses_side = zeros(0,3);
        end
        
        poses_points = [poses_points; poses_base; poses_top; poses_side];
        poses_prox = [poses_prox; poses_prox_base; poses_prox_top; poses_prox_side];
        d_min = [d_min; d_min_base; d_min_top; d_min_side];
        jacobians_prox = [jacobians_prox, jacobians_base, jacobians_top, jacobians_side];
        poses_base = zeros(0,3); poses_top = zeros(0,3); poses_side = zeros(0,3); poses_prox_base = zeros(0,3); poses_prox_top = zeros(0,3); poses_prox_side = zeros(0,3);
        d_min_base = zeros(0,1); d_min_top = zeros(0,1); d_min_side = zeros(0,1); jacobians_base = {}; jacobians_top = {}; jacobians_side = {};
        
    elseif strcmp(limit.limite(t).type,'tube')
        if ~isempty(poses_to_limit)
            base=limit.limite(t).surfaces.base;
            top=limit.limite(t).surfaces.base+limit.limite(t).surfaces.axe*limit.limite(t).surfaces.longueur;
            pos = linePosition3d(poses_to_limit', [base (top-base)]);
            
            pos_top = pos>1;
            poses_top = poses_to_limit(:,pos_top);
            jacobians_top = jacobians_to_limit(pos_top);
            
            pos_base = pos<0;
            poses_base = poses_to_limit(:,pos_base);
            jacobians_base = jacobians_to_limit(pos_base);
            
            pos_side = (pos>0 & pos<1);
            poses_side = poses_to_limit(:,pos_side);
            jacobians_side = jacobians_to_limit(pos_side);
            
            
            if ~isempty(poses_top) %si on est en haut de l'objet:
                
                p_A = top;
                vec = bsxfun(@minus,poses_top,p_A') - bsxfun(@times, (limit.limite(t).surfaces.axe) * bsxfun(@minus,poses_top,p_A'), (limit.limite(t).surfaces.axe)');
                ratio_diam = vecnorm(vec);
                ratio_diam(ratio_diam > limit.limite(t).surfaces.dia / 2) = limit.limite(t).surfaces.dia / 2;
                vec = vec .* ratio_diam ./ vecnorm(vec);
                poses_prox_top = (bsxfun(@plus, vec, p_A'))';
                d_min_top=vecnorm(poses_prox_top-poses_top',2, 2);
            end
            if ~isempty(poses_base)
                p_A = base;
                vec = bsxfun(@minus,poses_base,p_A') - bsxfun(@times, (limit.limite(t).surfaces.axe) * bsxfun(@minus,poses_base,p_A'), (limit.limite(t).surfaces.axe)');
                ratio_diam = vecnorm(vec);
                ratio_diam(ratio_diam > limit.limite(t).surfaces.dia / 2) = limit.limite(t).surfaces.dia / 2;
                vec = vec .* ratio_diam ./ vecnorm(vec);
                poses_prox_base = (bsxfun(@plus, vec, p_A'))';
                d_min_base=vecnorm(poses_prox_base-poses_base',2, 2);
            end
            if ~isempty(poses_side)
                p_A = poses_side;
                p_A(1,:) = base(1);
                p_A(2,:) = base(2);
                vec = poses_side-p_A;
                ratio_diam = vecnorm(vec);
                ratio_diam(ratio_diam > limit.limite(t).surfaces.dia / 2) = limit.limite(t).surfaces.dia / 2;
                vec = vec .* ratio_diam ./ vecnorm(vec);
                
                ignore = isnan(vec(1,:)) | (ratio_diam < limit.limite(t).surfaces.dia / 2);
                vec(:,ignore) = [];
                jacobians_side(ignore) = [];
                p_A(:,ignore) = [];
                poses_side(:,ignore) = [];
                
                poses_prox_side = (vec + p_A)';
                d_min_side=vecnorm(poses_prox_side-poses_side',2, 2);
            end
        poses_points = [poses_points; poses_base'; poses_top'; poses_side'];
        poses_prox = [poses_prox; poses_prox_base; poses_prox_top; poses_prox_side];
        d_min = [d_min; d_min_base; d_min_top; d_min_side];
        jacobians_prox = [jacobians_prox, jacobians_base, jacobians_top, jacobians_side];
        poses_base = zeros(0,3); poses_top = zeros(0,3); poses_side = zeros(0,3); poses_prox_base = zeros(0,3); poses_prox_top = zeros(0,3); poses_prox_side = zeros(0,3);
        d_min_base = zeros(0,1); d_min_top = zeros(0,1); d_min_side = zeros(0,1); jacobians_base = {}; jacobians_top = {}; jacobians_side = {};
        end
    elseif strcmp(limit.limite(t).type,'sphe')
        ratio_rayon = limit.limite(t).radius-vecnorm(bsxfun(@minus,poses_to_limit, (limit.limite(t).centroide)'));
        vec = bsxfun(@minus,poses_to_limit, (limit.limite(t).centroide)')./vecnorm(bsxfun(@minus,poses_to_limit, (limit.limite(t).centroide)'));
        poses_prox_shpere = (poses_to_limit + ratio_rayon * vec)';
        
        poses_prox = [poses_prox ; poses_prox_shpere];
        d_min = [d_min; vecnorm(poses_to_limit-poses_prox_shpere')];
        jacobians_prox = [jacobians_prox jacobians_to_limit];
        poses_points = [poses_points;poses_to_limit'];
    end
end
% pose_prox_act  sont les position des points de contacts

normales_effecteur = [];
vec_norm=(poses_points-poses_prox)./vecnorm(poses_points-poses_prox, 2,2);
for i = 1:size(vec_norm,1)
    normale_effecteur=PointToEffector_v6(vec_norm(i,:), jacobians_prox{i}, jacobian_eff);
    normales_effecteur = [normales_effecteur; normale_effecteur];
end
% pose_prox sont les position des points de contact reportés à l'effecteur
% poses_prox = pose_eff+normales_effecteur;
