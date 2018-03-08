    
function [normale_effecteur, collision_pose_eff, d_min, collision_poses, membrures_colisions] = collision_manager(limit, Robot_Pose_j, dh_eff, membrures_robot)



normale_effecteur = []; collision_poses = []; collision_pose_eff = []; d_min = []; membrures_colisions = [];


pose_prox=[];d_min=[];


z_offset = 0.899734982808; % pour tenir compte de l'élévation du robot dans le simulateur

t = 1;
[Robot_Poses, euler, poses_articulations] = cin_dir_6ddl_v3(Robot_Pose_j, dh_eff);
jacob_eff = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh_eff);



%On cherche si un objet entre en collision avec le robot

for i = 1:size(poses_articulations,2)
    dh_local = dh_UR5(i);
    jacob_local = jacob_UR5(Robot_Pose_j ,poses_articulations', dh_local, i);
    [d_min_t, pose_prox_t, pose_prox_pt_act]=verifDistance_v6(limit,pose,jacob_local, jacob_eff,Robot_Pose_j);
    d_min=[d_min d_min_t];
    pose_prox=[pose_prox;pose_prox_pt_act];
    poses_prox_pt_act.pose(i).poses=pose_prox_pt_act;
end


Robot_Poses = Robot_Poses(1:3,4)';
Robot_Poses = [Robot_Poses euler];
Robot_Poses(1, :);
jacob_eff = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh_eff);
for j=1:length(contacts.Contacts)
    try
        membrure = membrures_robot(contacts.Contacts(j).Collision2);
    catch
        continue
    end

    Pose_base_membrure = cin_dir_6ddl( Robot_Pose_j, dh_UR5(membrure-1));
    Pose_base_membrure = Pose_base_membrure(1:3,4)';
    Pose_bout_membrure = cin_dir_6ddl( Robot_Pose_j, dh_UR5(membrure));
    Pose_bout_membrure = Pose_bout_membrure(1:3,4)';
    Pose_collision = (contacts.Contacts(j).Position(1:3)' - [0, 0, z_offset]) * 1000;

    Pose_proj_collision = projPointOnLine3d(Pose_collision, [Pose_base_membrure,Pose_bout_membrure]);


    ratio = norm(Pose_proj_collision - Pose_base_membrure) / norm(Pose_bout_membrure - Pose_base_membrure);

    dh_local = dh_UR5(membrure, ratio);
    modifier = zeros(1,6);
    modifier(1:membrure) = 1;
    jacob_local = jacob_UR5(Robot_Pose_j ,Pose_collision, dh_local, membrure);

    normal = [-contacts.Contacts(j).Normal(1), -contacts.Contacts(j).Normal(2), -contacts.Contacts(j).Normal(3)];

    normale_effecteur_loc=PointToEffector_v4(normal, jacob_local, jacob_eff );


    normale_effecteur = [normale_effecteur; normale_effecteur_loc];
    collision_pose_eff(t,:) = Robot_Poses(1, :) - normale_effecteur_loc(1,1:6);
    collision_poses(t,:) = Pose_collision - normal;
    membrures_collisions(t) = [membrures_colisions membrure];
    d_min(t) = contacts.Contacts(j).Depth;
    t = t+1;
end

















