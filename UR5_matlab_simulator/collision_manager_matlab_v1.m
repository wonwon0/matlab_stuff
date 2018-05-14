    
function [normales_effecteur, d_min, pose_prox, h8, poses_articulations] = collision_manager_matlab_v1(limit, Robot_Pose_j, dh_eff, h8, min_distance)


poses_prox_pt_act = 1;
normales_effecteur = []; collision_poses = []; collision_pose_eff = []; d_min = []; membrures_colisions = [];


pose_prox=[];d_min=[];


z_offset = 0.899734982808; % pour tenir compte de l'élévation du robot dans le simulateur

t = 1;
[poses_articulations, jacobians] = get_interest_points_v1(Robot_Pose_j, dh_eff);
pose_eff = [poses_articulations(:,1)', 0, 0, 0];
jacob_eff = jacobians{1};


[d_min_t, poses_prox, poses_points, vect_normales_effecteur] = verifDistance_v8(limit,poses_articulations,jacobians, jacob_eff, pose_eff);
for j = 1:length(d_min_t)       %pour chacun des objets colisionables
        if d_min_t(j) < min_distance
            normales_effecteur = [normales_effecteur; vect_normales_effecteur(j,:)];
            d_min=[d_min d_min_t(j)];
            pose_prox=[pose_prox;poses_prox(j,:)];
        end
end
%On cherche si un objet entre en collision avec le robot
% for i = 1:size(poses_articulations,2)       %pour chacun des points d'intéret du robot
%     jacob_local = jacobians{i};
%     [d_min_t, ~, pose_prox_pt_act, vect_normales_effecteur]=verifDistance_v7(limit,poses_articulations(:,i)',jacob_local, jacob_eff, pose_eff);
%     
%     for j = 1:length(d_min_t)       %pour chacun des objets colisionables
%         if d_min_t(j) < min_distance
%             normales_effecteur = [normales_effecteur; vect_normales_effecteur(j,:)];
%             d_min=[d_min d_min_t(j)];
%             pose_prox=[pose_prox;pose_prox_pt_act(j,:)];
%         end
%         poses_prox_pt_act.pose(i).poses=pose_prox_pt_act;
%     end
% end

mat_ligne_x=[];
mat_ligne_y=[];
mat_ligne_z=[];
if 1
    for p=1:length(d_min_t)
        if isempty(poses_prox)
            continue
        end
        mat_ligne_x=[mat_ligne_x [poses_points(p,1) poses_prox(p,1)] NaN];
        mat_ligne_y=[mat_ligne_y [poses_points(p,2) poses_prox(p,2)] NaN];
        mat_ligne_z=[mat_ligne_z [poses_points(p,3) poses_prox(p,3)] NaN];
    end
    mat_ligne_x=mat_ligne_x(1:end-1);
    mat_ligne_y=mat_ligne_y(1:end-1);
    mat_ligne_z=mat_ligne_z(1:end-1);
    %h8=line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','r');
    h8.XData=mat_ligne_x;
    h8.YData=mat_ligne_y;
    h8.ZData=mat_ligne_z;
else
    t=9;
    mat_ligne_x=[NaN];
    mat_ligne_y=[NaN];
    mat_ligne_z=[NaN];
    h8.XData=mat_ligne_x;
    h8.YData=mat_ligne_y;
    h8.ZData=mat_ligne_z;
end






















