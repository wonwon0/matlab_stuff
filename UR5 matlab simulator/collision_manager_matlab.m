    
function [normales_effecteur, d_min, pose_prox, poses_prox_pt_act, h8] = collision_manager_matlab(limit, Robot_Pose_j, dh_eff, h8, min_distance)



normales_effecteur = []; collision_poses = []; collision_pose_eff = []; d_min = []; membrures_colisions = [];


pose_prox=[];d_min=[];


z_offset = 0.899734982808; % pour tenir compte de l'élévation du robot dans le simulateur

t = 1;
[Robot_Poses, euler, poses_articulations] = cin_dir_6ddl_v3(Robot_Pose_j, dh_eff);
pose_eff = [Robot_Poses(1:3, 4)', euler];
jacob_eff = jacob_UR5(Robot_Pose_j, Robot_Poses(1:3,4), dh_eff);



%On cherche si un objet entre en collision avec le robot
for i = 2:size(poses_articulations,2)
    dh_local = dh_UR5(i);
    jacob_local = jacob_UR5(Robot_Pose_j ,poses_articulations(:,i)', dh_local, i);
    [d_min_t, pose_prox_t, pose_prox_pt_act, vect_normales_effecteur]=verifDistance_v6(limit,poses_articulations(:,i)',jacob_local, jacob_eff, pose_eff, i);
    
    if d_min_t < min_distance
        normales_effecteur = [normales_effecteur; vect_normales_effecteur];
        d_min=[d_min d_min_t];
        pose_prox=[pose_prox;pose_prox_pt_act];
    end
    poses_prox_pt_act.pose(i).poses=pose_prox_pt_act;
end

mat_ligne_x=[];
mat_ligne_y=[];
mat_ligne_z=[];
if 1
    for p=1:length(poses_prox_pt_act.pose)
        t=size(poses_prox_pt_act.pose(p).poses,1);
        pose=poses_prox_pt_act.pose(p).poses';
        if isempty(pose)
            continue
        end
        for i=1:t
            mat_ligne_x=[mat_ligne_x [poses_articulations(1,p) pose(1,i)] NaN];
            mat_ligne_y=[mat_ligne_y [poses_articulations(2,p) pose(2,i)] NaN];
            mat_ligne_z=[mat_ligne_z [poses_articulations(3,p) pose(3,i)] NaN];
%                     h8(i+(p-1)*t)=line([pose_act(p,1) pose(i,1)],[pose_act(p,2) pose(i,2)],[pose_act(p,3) pose(i,3)],'LineWidth',2,'color','r');
        end
    end
    mat_ligne_x=mat_ligne_x(1:end-1);
    mat_ligne_y=mat_ligne_y(1:end-1);
    mat_ligne_z=mat_ligne_z(1:end-1);
%             h8=line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','r');
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









